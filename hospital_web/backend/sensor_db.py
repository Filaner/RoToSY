"""
SQLite persistence for Arduino temperature/humidity readings.

Tables:
  sensor_readings  — append-only measurement history
  drawer_sensors   — current state snapshot (upsert on each reading)

Thresholds (hospital pharmacy standard):
  Temperature: 15–25 °C
  Humidity:    40–70 %
"""

import sqlite3
import threading
import uuid
import random
import math
import os
import time
import logging
from datetime import datetime, timedelta
from pathlib import Path

DB_PATH = Path(__file__).parent / 'sensor_data.db'

TEMP_MIN, TEMP_MAX = 15.0, 25.0
HUMI_MIN, HUMI_MAX = 40.0, 70.0
SENSOR_ID = 'arduino_01'

_lock = threading.Lock()
_log  = logging.getLogger('sensor_db')

_serial_thread: threading.Thread | None = None
_serial_stop = threading.Event()


# ── Connection ────────────────────────────────────────────────────────────────

def _conn() -> sqlite3.Connection:
    c = sqlite3.connect(str(DB_PATH), check_same_thread=False)
    c.row_factory = sqlite3.Row
    return c


# ── Init ──────────────────────────────────────────────────────────────────────

def init_db() -> None:
    with _lock, _conn() as c:
        c.executescript('''
        PRAGMA journal_mode = WAL;

        CREATE TABLE IF NOT EXISTS sensor_readings (
            reading_id  TEXT PRIMARY KEY,
            sensor_id   TEXT NOT NULL DEFAULT 'arduino_01',
            temperature REAL NOT NULL,
            humidity    REAL NOT NULL,
            is_alert    INTEGER NOT NULL DEFAULT 0 CHECK (is_alert IN (0,1)),
            recorded_at TEXT NOT NULL,
            created_at  TEXT NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%S','now'))
        );

        CREATE INDEX IF NOT EXISTS idx_sr_recorded
            ON sensor_readings(recorded_at DESC);
        CREATE INDEX IF NOT EXISTS idx_sr_alert
            ON sensor_readings(is_alert) WHERE is_alert = 1;

        CREATE TABLE IF NOT EXISTS drawer_sensors (
            sensor_id              TEXT PRIMARY KEY,
            current_temperature    REAL,
            current_humidity       REAL,
            temp_threshold_min     REAL NOT NULL DEFAULT 15.0,
            temp_threshold_max     REAL NOT NULL DEFAULT 25.0,
            humidity_threshold_min REAL NOT NULL DEFAULT 40.0,
            humidity_threshold_max REAL NOT NULL DEFAULT 70.0,
            sensor_status          TEXT NOT NULL DEFAULT 'OFFLINE',
            last_reading_at        TEXT
        );

        INSERT OR IGNORE INTO drawer_sensors (sensor_id) VALUES ('arduino_01');
        ''')

    # 24h 시드 데이터가 없으면 생성
    if _count_24h() == 0:
        _seed_history()


# ── Write ─────────────────────────────────────────────────────────────────────

def insert_reading(temperature: float, humidity: float,
                   sensor_id: str = SENSOR_ID) -> dict:
    is_alert = int(
        not (TEMP_MIN <= temperature <= TEMP_MAX)
        or not (HUMI_MIN <= humidity <= HUMI_MAX)
    )
    reading_id  = f'R-{uuid.uuid4().hex[:8].upper()}'
    recorded_at = datetime.now().isoformat(timespec='seconds')

    with _lock, _conn() as c:
        c.execute(
            'INSERT INTO sensor_readings '
            '(reading_id, sensor_id, temperature, humidity, is_alert, recorded_at) '
            'VALUES (?,?,?,?,?,?)',
            (reading_id, sensor_id, temperature, humidity, is_alert, recorded_at)
        )
        status = 'WARNING' if is_alert else 'NORMAL'
        c.execute(
            '''UPDATE drawer_sensors SET
               current_temperature = ?, current_humidity = ?,
               sensor_status = ?, last_reading_at = ?
               WHERE sensor_id = ?''',
            (temperature, humidity, status, recorded_at, sensor_id)
        )

    return {
        'reading_id':  reading_id,
        'temperature': temperature,
        'humidity':    humidity,
        'is_alert':    bool(is_alert),
        'recorded_at': recorded_at,
    }


# ── Read ──────────────────────────────────────────────────────────────────────

def get_latest() -> dict | None:
    with _lock, _conn() as c:
        row = c.execute(
            '''SELECT ds.current_temperature, ds.current_humidity,
                      ds.sensor_status, ds.last_reading_at,
                      ds.temp_threshold_min, ds.temp_threshold_max,
                      ds.humidity_threshold_min, ds.humidity_threshold_max
               FROM drawer_sensors ds WHERE ds.sensor_id = ?''',
            (SENSOR_ID,)
        ).fetchone()
    if not row or row[0] is None:
        return None
    return {
        'temperature':   row[0],
        'humidity':      row[1],
        'status':        row[2],
        'last_seen':     row[3],
        'temp_min':      row[4],
        'temp_max':      row[5],
        'humi_min':      row[6],
        'humi_max':      row[7],
        'is_alert':      row[2] == 'WARNING',
    }


def get_history(hours: int = 24) -> list[dict]:
    since = (datetime.now() - timedelta(hours=hours)).isoformat(timespec='seconds')
    with _lock, _conn() as c:
        rows = c.execute(
            '''SELECT temperature, humidity, is_alert, recorded_at
               FROM sensor_readings
               WHERE recorded_at >= ? ORDER BY recorded_at ASC''',
            (since,)
        ).fetchall()
    return [
        {'temperature': r[0], 'humidity': r[1],
         'is_alert': bool(r[2]), 'recorded_at': r[3]}
        for r in rows
    ]


def get_alert_count(hours: int = 24) -> int:
    since = (datetime.now() - timedelta(hours=hours)).isoformat(timespec='seconds')
    with _lock, _conn() as c:
        row = c.execute(
            'SELECT COUNT(*) FROM sensor_readings WHERE recorded_at >= ? AND is_alert = 1',
            (since,)
        ).fetchone()
    return row[0] if row else 0


def get_thresholds() -> dict:
    with _lock, _conn() as c:
        row = c.execute(
            '''SELECT temp_threshold_min, temp_threshold_max,
                      humidity_threshold_min, humidity_threshold_max
               FROM drawer_sensors WHERE sensor_id = ?''',
            (SENSOR_ID,)
        ).fetchone()
    if not row:
        return {'temp_min':15.0,'temp_max':25.0,'humi_min':40.0,'humi_max':70.0}
    return {'temp_min':row[0],'temp_max':row[1],'humi_min':row[2],'humi_max':row[3]}


def update_thresholds(temp_min: float, temp_max: float,
                      humi_min: float, humi_max: float) -> None:
    with _lock, _conn() as c:
        c.execute(
            '''UPDATE drawer_sensors SET
               temp_threshold_min=?, temp_threshold_max=?,
               humidity_threshold_min=?, humidity_threshold_max=?
               WHERE sensor_id=?''',
            (temp_min, temp_max, humi_min, humi_max, SENSOR_ID)
        )


# ── Helpers ───────────────────────────────────────────────────────────────────

def _count_24h() -> int:
    since = (datetime.now() - timedelta(hours=24)).isoformat(timespec='seconds')
    with _lock, _conn() as c:
        row = c.execute(
            'SELECT COUNT(*) FROM sensor_readings WHERE recorded_at >= ?', (since,)
        ).fetchone()
    return row[0] if row else 0


def _seed_history() -> None:
    """24시간 시드 데이터 생성 (5분 간격, 288건)."""
    now  = datetime.now()
    base_temp = 20.5
    base_humi = 55.0
    rows = []
    for i in range(288):
        t = now - timedelta(minutes=5 * (287 - i))
        # 자연스러운 일주기 변화 (sin wave)
        hour_factor = math.sin(t.hour * math.pi / 12) * 1.5
        temp = round(base_temp + hour_factor + random.gauss(0, 0.3), 1)
        humi = round(base_humi - hour_factor * 1.2 + random.gauss(0, 1.0), 1)
        temp = max(14.0, min(27.0, temp))
        humi = max(35.0, min(75.0, humi))
        is_alert = int(not (TEMP_MIN<=temp<=TEMP_MAX) or not (HUMI_MIN<=humi<=HUMI_MAX))
        rows.append((
            f'R-SEED{i:04d}', SENSOR_ID, temp, humi, is_alert,
            t.isoformat(timespec='seconds')
        ))

    with _lock, _conn() as c:
        c.executemany(
            'INSERT OR IGNORE INTO sensor_readings '
            '(reading_id, sensor_id, temperature, humidity, is_alert, recorded_at) '
            'VALUES (?,?,?,?,?,?)',
            rows
        )
        last = rows[-1]
        status = 'WARNING' if last[4] else 'NORMAL'
        c.execute(
            '''UPDATE drawer_sensors SET current_temperature=?, current_humidity=?,
               sensor_status=?, last_reading_at=? WHERE sensor_id=?''',
            (last[2], last[3], status, last[5], SENSOR_ID)
        )


# ── Serial reader (background thread) ────────────────────────────────────────
#
# Arduino DHT22 → CSV "t,h\n" 라인을 읽어 insert_reading()로 적재.
# Env: SENSOR_PORT (기본 /dev/ttyACM0), SENSOR_BAUD (9600), SENSOR_ENABLED (1)
# 시리얼 열기 실패 / 권한 거부 / 라인 깨짐 등은 모두 graceful — 백엔드를 죽이지 않음.

def start_serial_reader() -> None:
    global _serial_thread
    if _serial_thread and _serial_thread.is_alive():
        return
    if os.environ.get('SENSOR_ENABLED', '1') == '0':
        _log.info('sensor reader disabled (SENSOR_ENABLED=0)')
        return
    _serial_stop.clear()
    _serial_thread = threading.Thread(
        target=_serial_loop, name='sensor-serial', daemon=True
    )
    _serial_thread.start()


def stop_serial_reader() -> None:
    _serial_stop.set()
    if _serial_thread and _serial_thread.is_alive():
        _serial_thread.join(timeout=2.0)


def _serial_loop() -> None:
    port = os.environ.get('SENSOR_PORT', '/dev/ttyACM0')
    baud = int(os.environ.get('SENSOR_BAUD', '9600'))

    try:
        import serial   # pyserial; lazy import
    except ImportError:
        _log.warning('pyserial 미설치 — sensor reader 비활성')
        return

    while not _serial_stop.is_set():
        try:
            with serial.Serial(port, baud, timeout=2) as ser:
                _log.info(f'sensor reader: connected {port} @ {baud}, Arduino 리셋 대기 ~3s')
                # Arduino DTR 리셋 후 첫 측정까지 정적 구간
                if _serial_stop.wait(3.0):
                    return
                ser.reset_input_buffer()

                while not _serial_stop.is_set():
                    line = ser.readline().decode(errors='ignore').strip()
                    if not line:
                        continue
                    try:
                        t_str, h_str = line.split(',')
                        t, h = float(t_str), float(h_str)
                    except ValueError:
                        continue
                    if math.isnan(t) or math.isnan(h):
                        continue
                    try:
                        insert_reading(t, h)
                    except Exception as e:
                        _log.warning(f'insert_reading 실패: {e}')

        except PermissionError as e:
            _log.warning(
                f'sensor reader: permission denied on {port} '
                f'(dialout 권한 또는 sudo 필요) — {e}; 5s 뒤 재시도'
            )
        except Exception as e:
            _log.warning(
                f'sensor reader: {type(e).__name__} on {port} — {e}; 5s 뒤 재시도'
            )

        # interruptible 5s backoff
        if _serial_stop.wait(5.0):
            return
