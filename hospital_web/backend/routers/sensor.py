"""Arduino 온습도 API."""

from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel

from .. import sensor_db as db
from .. import mission_state as ms

router = APIRouter(prefix='/api/sensor', tags=['sensor'])


class ReadingReq(BaseModel):
    temperature: float
    humidity:    float
    sensor_id:   str = 'arduino_01'


class ThresholdReq(BaseModel):
    temp_min: float = 15.0
    temp_max: float = 25.0
    humi_min: float = 40.0
    humi_max: float = 70.0


@router.get('/current')
async def current():
    data = db.get_latest()
    if data is None:
        return {'status': 'OFFLINE', 'temperature': None, 'humidity': None}
    data['alert_count_24h'] = db.get_alert_count(24)
    return data


@router.get('/history')
async def history(hours: int = Query(default=24, ge=1, le=168)):
    return db.get_history(hours)


@router.get('/alerts')
async def alerts(hours: int = Query(default=24, ge=1, le=168)):
    count = db.get_alert_count(hours)
    recent = [r for r in db.get_history(hours) if r['is_alert']][-20:]
    return {'count': count, 'recent': recent}


@router.post('/reading')
async def push_reading(req: ReadingReq):
    """Arduino HTTP push 모드 — Arduino가 직접 이 엔드포인트로 POST."""
    if not (-10 <= req.temperature <= 60):
        raise HTTPException(status_code=422, detail='temperature out of range')
    if not (0 <= req.humidity <= 100):
        raise HTTPException(status_code=422, detail='humidity out of range')
    result = db.insert_reading(req.temperature, req.humidity, req.sensor_id)
    if result['is_alert']:
        ms.add_audit('system', 'SENSOR_ALERT',
                     f'온도 {req.temperature}°C / 습도 {req.humidity}% — 임계값 초과')
    return result


@router.get('/thresholds')
async def get_thresholds():
    return db.get_thresholds()


@router.put('/thresholds')
async def update_thresholds(req: ThresholdReq):
    if req.temp_min >= req.temp_max:
        raise HTTPException(status_code=422, detail='temp_min must be < temp_max')
    if req.humi_min >= req.humi_max:
        raise HTTPException(status_code=422, detail='humi_min must be < humi_max')
    db.update_thresholds(req.temp_min, req.temp_max, req.humi_min, req.humi_max)
    ms.add_audit('admin', 'THRESHOLD_UPDATE',
                 f'온도 {req.temp_min}~{req.temp_max}°C / 습도 {req.humi_min}~{req.humi_max}%')
    return db.get_thresholds()
