"""
OCR 인증 로직.

verify_and_save(ocr_result)
  - 현재 활성 미션의 prescription_item 목록과 OCR 결과를 매칭
  - 결과를 ocr_scan 테이블에 저장하고 audit_log에 기록
  - 반환: { match_status, matched_item, mismatch_reason, scan_id, pending_items }

get_scans(mission_code)   — 미션의 전체 스캔 이력
get_pending_items(mission_code) — 아직 MATCHED 되지 않은 처방 품목
"""

import json
import threading
from datetime import datetime
from typing import Optional

from .db_schema import get_conn
from . import mission_state as ms

_lock = threading.Lock()


def _now() -> str:
    return datetime.now().isoformat(timespec='seconds')


def _normalize(text: str) -> str:
    """비교용 정규화: 소문자, 공백·특수문자 제거."""
    return ''.join(c for c in text.lower() if c.isalnum())


def _match_name(detected: str, expected: str) -> bool:
    """약품명 매칭: 정규화 후 exact → contains 순으로 시도."""
    d = _normalize(detected)
    e = _normalize(expected)
    if not d or not e:
        return False
    return d == e or d in e or e in d


# ── 핵심 함수 ─────────────────────────────────────────────────────────────────

def verify_and_save(ocr_result: dict) -> dict:
    """
    ocr_result: Groq llama-4-scout가 반환한 dict
      { medicine_name, dosage, instructions, patient_name,
        prescription_date, ward, raw_text }

    반환:
      { scan_id, match_status, matched_item, mismatch_reason,
        pending_count, all_matched }
    """
    mission = ms.get_mission()
    mission_code = mission.get('mission_id')
    pres_code    = mission.get('prescription_id')

    detected_name = (ocr_result.get('medicine_name') or '').strip()
    detected_dose = (ocr_result.get('dosage') or '').strip()
    raw_text      = (ocr_result.get('raw_text') or '').strip()

    with _lock, get_conn() as c:
        # mission / prescription int id 조회
        mission_int_id = None
        pres_int_id    = None

        if mission_code:
            row = c.execute('SELECT id FROM mission WHERE code=?',
                            (mission_code,)).fetchone()
            if row:
                mission_int_id = row['id']

        if pres_code:
            row = c.execute('SELECT id FROM prescription WHERE code=?',
                            (pres_code,)).fetchone()
            if row:
                pres_int_id = row['id']

        # 처방 품목 목록 조회
        items = []
        if pres_int_id:
            items = c.execute(
                '''SELECT id, medicine_name, quantity, frequency, dosage
                   FROM prescription_item
                   WHERE prescription_id = ?
                   ORDER BY sort_order, id''',
                (pres_int_id,)
            ).fetchall()

        # 매칭
        match_status    = 'UNKNOWN'
        matched_item_id = None
        matched_item    = None
        mismatch_reason = ''

        if not items:
            mismatch_reason = '처방 품목 없음 (미션 또는 처방 미연결)'
            match_status = 'UNKNOWN'
        elif not detected_name:
            mismatch_reason = 'OCR에서 약품명 미감지'
            match_status = 'UNKNOWN'
        else:
            for item in items:
                if _match_name(detected_name, item['medicine_name']):
                    matched_item_id = item['id']
                    matched_item = {
                        'id':            item['id'],
                        'medicine_name': item['medicine_name'],
                        'quantity':      item['quantity'],
                        'frequency':     item['frequency'],
                        'dosage':        item['dosage'],
                    }
                    match_status = 'MATCHED'
                    break

            if match_status != 'MATCHED':
                expected_names = [i['medicine_name'] for i in items]
                mismatch_reason = (
                    f'감지된 약품 "{detected_name}"이 '
                    f'처방 목록 {expected_names}에 없음'
                )
                match_status = 'MISMATCH'

        # ocr_scan 저장
        c.execute(
            '''INSERT INTO ocr_scan
               (mission_id, prescription_id, medicine_name, dosage,
                raw_text, ocr_json, match_status, matched_item_id, mismatch_reason,
                scanned_at)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)''',
            (mission_int_id, pres_int_id,
             detected_name, detected_dose, raw_text,
             json.dumps(ocr_result, ensure_ascii=False),
             match_status, matched_item_id, mismatch_reason,
             _now())
        )
        scan_id = c.execute('SELECT last_insert_rowid()').fetchone()[0]

        # audit_log 기록
        detail = (
            f'스캔#{scan_id} [{match_status}] '
            f'감지={detected_name or "??"} '
            f'미션={mission_code or "없음"}'
        )
        if mismatch_reason:
            detail += f' — {mismatch_reason}'
        c.execute(
            '''INSERT INTO audit_log (mission_id, actor, action, detail, created_at)
               VALUES (?, ?, ?, ?, ?)''',
            (mission_int_id, 'robot', 'OCR_SCAN', detail, _now())
        )

        # 남은 미매칭 품목 계산
        matched_ids = {
            r['matched_item_id']
            for r in c.execute(
                '''SELECT matched_item_id FROM ocr_scan
                   WHERE mission_id=? AND match_status='MATCHED'
                   AND matched_item_id IS NOT NULL''',
                (mission_int_id,)
            ).fetchall()
        }
        pending = [
            {'id': i['id'], 'medicine_name': i['medicine_name'],
             'quantity': i['quantity']}
            for i in items if i['id'] not in matched_ids
        ]

    return {
        'scan_id':        scan_id,
        'match_status':   match_status,
        'matched_item':   matched_item,
        'mismatch_reason': mismatch_reason,
        'pending_count':  len(pending),
        'all_matched':    len(pending) == 0 and len(items) > 0,
        'pending_items':  pending,
    }


def get_scans(mission_code: str) -> list:
    """미션의 전체 스캔 이력 (최신순)."""
    with _lock, get_conn() as c:
        row = c.execute('SELECT id FROM mission WHERE code=?',
                        (mission_code,)).fetchone()
        if not row:
            return []
        rows = c.execute(
            '''SELECT id, medicine_name, dosage, raw_text, ocr_json,
                      match_status, mismatch_reason, scanned_at
               FROM ocr_scan
               WHERE mission_id=?
               ORDER BY scanned_at DESC''',
            (row['id'],)
        ).fetchall()
    return [
        {
            'scan_id':        r['id'],
            'medicine_name':  r['medicine_name'],
            'dosage':         r['dosage'],
            'raw_text':       r['raw_text'],
            'ocr_json':       json.loads(r['ocr_json'] or '{}'),
            'match_status':   r['match_status'],
            'mismatch_reason': r['mismatch_reason'],
            'scanned_at':     r['scanned_at'],
        }
        for r in rows
    ]


def get_pending_items(mission_code: str) -> list:
    """아직 MATCHED 되지 않은 처방 품목."""
    with _lock, get_conn() as c:
        m = c.execute('SELECT id, prescription_id FROM mission WHERE code=?',
                      (mission_code,)).fetchone()
        if not m or not m['prescription_id']:
            return []

        items = c.execute(
            '''SELECT id, medicine_name, quantity, frequency, dosage
               FROM prescription_item
               WHERE prescription_id=?
               ORDER BY sort_order, id''',
            (m['prescription_id'],)
        ).fetchall()

        matched_ids = {
            r['matched_item_id']
            for r in c.execute(
                '''SELECT matched_item_id FROM ocr_scan
                   WHERE mission_id=? AND match_status='MATCHED'
                   AND matched_item_id IS NOT NULL''',
                (m['id'],)
            ).fetchall()
        }

    return [
        {
            'id':            i['id'],
            'medicine_name': i['medicine_name'],
            'quantity':      i['quantity'],
            'frequency':     i['frequency'],
            'dosage':        i['dosage'],
        }
        for i in items if i['id'] not in matched_ids
    ]
