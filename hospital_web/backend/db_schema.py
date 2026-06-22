"""
Centralized schema for hospital_web domain (new_erd.md v2 기반).

Same SQLite file as sensor_db.py (hospital.db).
Tables managed here:
  ward, staff, patient, medicine,
  prescription, prescription_item,
  cabinet, cabinet_slot,
  mission, audit_log

NOTE: sensor_readings / drawer_sensors는 sensor_db.py가 그대로 소유.
      Sensor / SensorReading 정식 마이그레이션은 별도 단계.

공용 get_conn()이 PRAGMA foreign_keys / busy_timeout / row_factory 일괄 설정.
"""

import sqlite3
import threading
from pathlib import Path

DB_PATH = Path(__file__).parent / 'hospital.db'
_lock = threading.Lock()


def get_conn() -> sqlite3.Connection:
    c = sqlite3.connect(str(DB_PATH), check_same_thread=False)
    c.row_factory = sqlite3.Row
    c.execute('PRAGMA foreign_keys = ON')
    c.execute('PRAGMA busy_timeout = 5000')
    return c


def init_schema() -> None:
    with _lock, get_conn() as c:
        c.executescript('''
        PRAGMA journal_mode = WAL;

        CREATE TABLE IF NOT EXISTS ward (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            name        TEXT NOT NULL UNIQUE,
            location    TEXT,
            goal_x      REAL,
            goal_y      REAL,
            goal_theta  REAL DEFAULT 0
        );

        CREATE TABLE IF NOT EXISTS staff (
            id              INTEGER PRIMARY KEY AUTOINCREMENT,
            name            TEXT NOT NULL,
            role            TEXT NOT NULL
                            CHECK (role IN ('DOCTOR','NURSE','PHARMACIST','ADMIN')),
            login_id        TEXT UNIQUE,
            password_hash   TEXT
        );

        CREATE TABLE IF NOT EXISTS patient (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            ward_id     INTEGER REFERENCES ward(id),
            name        TEXT NOT NULL,
            chart_no    TEXT UNIQUE,
            bed_no      TEXT
        );

        CREATE TABLE IF NOT EXISTS medicine (
            id              INTEGER PRIMARY KEY AUTOINCREMENT,
            name            TEXT NOT NULL UNIQUE,
            display_name    TEXT,
            width           REAL,
            height          REAL,
            depth           REAL,
            img_path        TEXT,
            barcode_plane   TEXT
        );

        CREATE TABLE IF NOT EXISTS prescription (
            id                      INTEGER PRIMARY KEY AUTOINCREMENT,
            code                    TEXT UNIQUE NOT NULL,
            patient_id              INTEGER REFERENCES patient(id),
            doctor_id               INTEGER REFERENCES staff(id),
            priority                TEXT NOT NULL DEFAULT 'general',
            status                  TEXT NOT NULL DEFAULT 'pending',
            pharmacist_note         TEXT DEFAULT '',
            reject_reason           TEXT DEFAULT '',
            ocr_raw                 TEXT DEFAULT '',
            ocr_confidence          REAL DEFAULT 0,
            ocr_parsed              TEXT DEFAULT '[]',
            vision_data             TEXT DEFAULT '[]',
            delivery_requested      INTEGER NOT NULL DEFAULT 0,
            delivery_requested_at   TEXT,
            prescribed_at           TEXT,
            created_at              TEXT NOT NULL
                                    DEFAULT (strftime('%Y-%m-%dT%H:%M:%S','now')),
            updated_at              TEXT NOT NULL
                                    DEFAULT (strftime('%Y-%m-%dT%H:%M:%S','now'))
        );

        CREATE TABLE IF NOT EXISTS prescription_item (
            id              INTEGER PRIMARY KEY AUTOINCREMENT,
            prescription_id INTEGER NOT NULL
                            REFERENCES prescription(id) ON DELETE CASCADE,
            medicine_id     INTEGER REFERENCES medicine(id),
            medicine_name   TEXT NOT NULL,
            quantity        INTEGER NOT NULL DEFAULT 1,
            frequency       TEXT DEFAULT '',
            dosage          TEXT DEFAULT '',
            sort_order      INTEGER DEFAULT 0
        );

        CREATE TABLE IF NOT EXISTS cabinet (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            code        TEXT UNIQUE NOT NULL,
            location    TEXT,
            magnet_x    REAL DEFAULT 0,
            magnet_y    REAL DEFAULT 0,
            magnet_z    REAL DEFAULT 0,
            size_x      REAL DEFAULT 0,
            size_y      REAL DEFAULT 0,
            size_z      REAL DEFAULT 0
        );

        CREATE TABLE IF NOT EXISTS cabinet_slot (
            id              INTEGER PRIMARY KEY AUTOINCREMENT,
            cabinet_id      INTEGER NOT NULL
                            REFERENCES cabinet(id) ON DELETE CASCADE,
            medicine_id     INTEGER REFERENCES medicine(id),
            code            TEXT UNIQUE NOT NULL,
            row_idx         INTEGER NOT NULL,
            col_idx         INTEGER NOT NULL,
            aruco_marker_id INTEGER UNIQUE,
            label           TEXT,
            pixel_x         INTEGER DEFAULT 0,
            pixel_y         INTEGER DEFAULT 0,
            max_capacity    INTEGER DEFAULT 10,
            current_stock   INTEGER DEFAULT 0
        );

        CREATE TABLE IF NOT EXISTS mission (
            id                      INTEGER PRIMARY KEY AUTOINCREMENT,
            code                    TEXT UNIQUE NOT NULL,
            prescription_id         INTEGER REFERENCES prescription(id),
            destination             TEXT,
            status                  TEXT NOT NULL DEFAULT 'IDLE',
            pharmacist_confirmed    INTEGER NOT NULL DEFAULT 0,
            admin_confirmed         INTEGER NOT NULL DEFAULT 0,
            created_at              TEXT NOT NULL
                                    DEFAULT (strftime('%Y-%m-%dT%H:%M:%S','now')),
            confirmed_at            TEXT,
            dispatched_at           TEXT,
            arrived_at              TEXT,
            completed_at            TEXT
        );

        CREATE TABLE IF NOT EXISTS audit_log (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            mission_id  INTEGER REFERENCES mission(id),
            actor       TEXT NOT NULL,
            action      TEXT NOT NULL,
            detail      TEXT DEFAULT '',
            created_at  TEXT NOT NULL
                        DEFAULT (strftime('%Y-%m-%dT%H:%M:%S','now'))
        );

        CREATE TABLE IF NOT EXISTS ocr_scan (
            id              INTEGER PRIMARY KEY AUTOINCREMENT,
            mission_id      INTEGER REFERENCES mission(id),
            prescription_id INTEGER REFERENCES prescription(id),
            medicine_name   TEXT    DEFAULT '',
            dosage          TEXT    DEFAULT '',
            raw_text        TEXT    DEFAULT '',
            ocr_json        TEXT    DEFAULT '{}',
            match_status    TEXT    NOT NULL DEFAULT 'UNKNOWN'
                            CHECK (match_status IN ('MATCHED','MISMATCH','UNKNOWN')),
            matched_item_id INTEGER REFERENCES prescription_item(id),
            mismatch_reason TEXT    DEFAULT '',
            scanned_at      TEXT    NOT NULL
                            DEFAULT (strftime('%Y-%m-%dT%H:%M:%S','now'))
        );

        CREATE TABLE IF NOT EXISTS orchestrator_state (
            id              INTEGER PRIMARY KEY CHECK (id = 1),
            state_json      TEXT NOT NULL,
            updated_at      TEXT NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%S','now'))
        );

        CREATE INDEX IF NOT EXISTS idx_prescription_status ON prescription(status);
        CREATE INDEX IF NOT EXISTS idx_prescription_created ON prescription(created_at);
        CREATE INDEX IF NOT EXISTS idx_pitem_pid ON prescription_item(prescription_id);
        CREATE INDEX IF NOT EXISTS idx_audit_created ON audit_log(created_at DESC);
        CREATE INDEX IF NOT EXISTS idx_audit_mission ON audit_log(mission_id);
        CREATE INDEX IF NOT EXISTS idx_slot_cab ON cabinet_slot(cabinet_id);
        CREATE INDEX IF NOT EXISTS idx_mission_created ON mission(created_at DESC);
        CREATE INDEX IF NOT EXISTS idx_scan_mission ON ocr_scan(mission_id);
        CREATE INDEX IF NOT EXISTS idx_scan_scanned ON ocr_scan(scanned_at DESC);
        ''')
