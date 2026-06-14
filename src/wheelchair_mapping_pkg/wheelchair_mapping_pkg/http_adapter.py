#!/usr/bin/env python3
"""HTTP adapter microservice to accept wheelchair updates and write to Firestore.
"""
import logging
import os
import json
from typing import Optional, Dict

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


class Location(BaseModel):
    x: float
    y: float


class WheelchairUpdate(BaseModel):
    id: str
    location: Location
    status: str
    updatedAt: Optional[Dict] = None


try:
    import firebase_admin
    from firebase_admin import credentials, firestore

    cred_path = os.environ.get('FIREBASE_CREDENTIAL', '/path/to/serviceAccountKey.json')
    if os.path.exists(cred_path):
        cred = credentials.Certificate(cred_path)
        firebase_admin.initialize_app(cred)
        logger.info('Initialized firebase-admin with %s', cred_path)
    else:
        try:
            firebase_admin.initialize_app()
            logger.info('Initialized firebase-admin using default credentials')
        except Exception:
            raise

    db = firestore.client()
    SERVER_TIMESTAMP = firestore.SERVER_TIMESTAMP
    _FIREBASE = True
except Exception as e:
    logger.warning('firebase_admin unavailable: %s', e)

    class _MockDoc:
        def __init__(self, doc_id):
            self.doc_id = doc_id

        def set(self, data, merge=False):
            logger.info('[firestore-mock] set %s -> %s (merge=%s)', self.doc_id, data, merge)

        def update(self, data):
            logger.info('[firestore-mock] update %s -> %s', self.doc_id, data)

    class _MockCollection:
        def __init__(self, name):
            self.name = name

        def document(self, doc_id):
            return _MockDoc(doc_id)

    class _MockDB:
        def collection(self, name):
            return _MockCollection(name)

    db = _MockDB()
    SERVER_TIMESTAMP = None
    _FIREBASE = False


app = FastAPI(title='Wheelchair Firestore Adapter')


@app.post('/update')
async def update_wheelchair(payload: WheelchairUpdate):
    doc_ref = db.collection('wheelchairs').document(payload.id)
    data = {
        'location': {'x': payload.location.x, 'y': payload.location.y},
        'status': payload.status,
    }
    if SERVER_TIMESTAMP is not None:
        data['updatedAt'] = SERVER_TIMESTAMP
    else:
        data['updatedAt'] = payload.updatedAt or {}

    try:
        if hasattr(doc_ref, 'set'):
            doc_ref.set(data, merge=True)
        else:
            doc_ref.update(data)
    except Exception as e:
        logger.exception('Failed writing to Firestore: %s', e)
        raise HTTPException(status_code=500, detail='failed to write to firestore')

    return {'ok': True}


if __name__ == '__main__':
    import uvicorn
    port = int(os.environ.get('PORT', '8080'))
    logger.info('Starting http adapter on 0.0.0.0:%d (firebase=%s)', port, _FIREBASE)
    uvicorn.run('wheelchair_mapping_pkg.http_adapter:app', host='0.0.0.0', port=port, log_level='info')
