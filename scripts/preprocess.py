#!/usr/bin/env python3
"""
Scripts to drive a donkey 2 car

Usage:
    preprocess.py [--tubs=tubs]

Options:
    -h --help               Show this screen.
"""

import os
import time
import logging
import json
from pathlib import Path
from donkeycar.parts.tub_v2 import Tub
from donkeycar.parts.datastore import TubHandler
from donkeycar.pipeline.types import TubRecord

from donkeycar.management.tub import TubManager
from donkeycar.pipeline.types import TubDataset

from docopt import docopt

import donkeycar as dk


class tubPreProcessor ():

    def __init__(self,
                 tub_paths: str) -> None:

        self.file_path = tub_paths
        self.tub = Tub(self.file_path)

    def getRecords(self):

        expression = None

        def select(underlying):
            if not expression:
                return True
            else:
                try:
                    record = TubRecord(cfg, self.tub.base_path, underlying)
                    res = eval(expression)
                    return res
                except KeyError as err:
                    Logger.error(f'Filter: {err}')
                    return True

        self.records = [TubRecord(cfg, self.tub.base_path, record)
                        for record in self.tub if select(record)]
        self.len = len(self.records)

        if self.len > 0:
            msg = f'Loaded tub {self.file_path} with {self.len} records'
            print (msg)

        ANGLE='user/angle'
        STRAIGHT='aug/straight'
        BRAKE_SIZE = 20
        ACCEL_SIZE = 20
        STEERING_THRESH = 10
        ACCEL_THRESH = 100
        idx_start_of_brake = -1
        idx_start_of_accel = -1
        brakeZoneCounter = 0
        accelZoneCounter = 0

        # Detect straight lines
        for idx, rec in enumerate(self.records):
            if (idx_start_of_brake == -1):
                if (abs(rec.underlying[ANGLE]) > 0.2):
                    # start of turn
                    idx_start_of_brake = idx
                    print ("New turn detected @ " + str(idx_start_of_brake))

            if (idx_start_of_accel == -1):
                if (abs(rec.underlying[ANGLE]) < 0.1):
                    # start of accel
                    idx_start_of_accel = idx
                    print ("New straight run detected @ " + str(idx_start_of_accel))

            if ((abs(rec.underlying[ANGLE]) > 0.1) and (idx_start_of_accel != -1)):
                # end of accell
                print ("End of straight run detected " + str(idx_start_of_accel))
                if ((idx-idx_start_of_accel) > ACCEL_THRESH):
                    # it is a long run
                    print("Long run detected @ "+str(idx_start_of_accel))
                    if (idx_start_of_accel>ACCEL_SIZE):
                        for repl in range(ACCEL_SIZE):
                            self.records[idx_start_of_accel+repl].underlying[STRAIGHT] = 2
                        accelZoneCounter=accelZoneCounter+1
                    idx_start_of_accel=-1

            if ((abs(rec.underlying[ANGLE]) < 0.2) and (idx_start_of_brake != -1)):
                # end of turn
                print ("End of turn detected " + str(idx_start_of_brake))
                if ((idx-idx_start_of_brake) > STEERING_THRESH):
                    # it is a large turn
                    print("Large turn detected @ "+str(idx_start_of_brake))
                    if (idx_start_of_brake>BRAKE_SIZE):
                        for repl in range(BRAKE_SIZE):
                            self.records[idx_start_of_brake-BRAKE_SIZE+repl].underlying[STRAIGHT] = 2
                        brakeZoneCounter=brakeZoneCounter+1
                    idx_start_of_brake=-1


class tubPreProcessor2:
    def __init__(self,
                tub_paths: str, cfg) -> None:
        base_path = Path(os.path.expanduser(tub_paths)).absolute().as_posix()
        self.dataset = TubDataset(config=cfg, tub_paths=[base_path],
                        seq_size=20)
                    
    def processRecords(self) -> None:
        self.dataset.get_records()

class tubPreProcessor3:
    def __init__(self,
                tub_paths: str, cfg) -> None:
        base_path = Path(os.path.expanduser(tub_paths)).absolute().as_posix()
        self.tubhandler = TubHandler(path=cfg.DATA_PATH)
        tubs = self.tubhandler.get_tub_list(cfg.DATA_PATH)
        print (tubs)

    def processRecords(self) -> None:
        pass

class tubPreProcessor4:

    ANGLE='user/angle'
    LONG_RUN='aug/run'
    LONG_TURN='aug/turn'
    LOCATION='localizer/location'
    BRAKE_SIZE = 20
    LONG_ACCEL_LENGTH = 50
    LONG_ACCEL_THRESH = 0.4
    LONG_TURN_LENGTH = 10
    LONG_TURN_THRESH = 0.4
    ANTICIPATION = 10

    def __init__(self,
                tub_paths: str, cfg) -> None:

        inputs=['cam/image_array','user/angle', 'user/throttle', 'user/mode']
        types=['image_array','float', 'float','str']
        inputs += ['enc/speed']
        types += ['float']
        meta=[]
        self.tub = Tub(base_path=cfg.DATA_PATH, inputs=inputs, types=types, metadata=meta)
        self.cfg = cfg

    def tagRecords(self, collection, start, end, tag, value):
        print (f"Tag from {start} to {end} tag {value}")
        for idx, record in enumerate(collection):
            if idx>=start and idx<=end:
                record[tag]=value

    def processRecords(self) -> None:
        tub_full = Tub(base_path=self.cfg.DATA_PATH+"/../data_processed",
                inputs=self.tub.manifest.inputs +
                [self.LONG_RUN, self.LONG_TURN, self.LOCATION],
                types=self.tub.manifest.types + ['int', 'int', 'int'])

        print (tub_full.manifest.inputs)
        print (tub_full.manifest.types)

        idx_start_of_brake = -1
        idx_start_of_accel = -1
        brakeZoneCounter = 0
        accelZoneCounter = 0

        newRecords=[]

        for idx, record in enumerate(self.tub):
            t = TubRecord(self.cfg, self.tub.base_path, record)
            img = t.image()

            newRecord = t.underlying
            newRecord['cam/image_array'] = img
            newRecord[self.LONG_TURN] = 0
            newRecord[self.LONG_RUN] = 0
            newRecord[self.LOCATION] = 0
            newRecords.append(newRecord)
            
            # Detect straight lines
            angle = abs(record[self.ANGLE])
            if (idx_start_of_accel == -1):
                if (angle <= self.LONG_ACCEL_THRESH):
                    # start of accel
                    idx_start_of_accel = idx
                    print ("New straight run detected @ " + str(idx_start_of_accel))

            if ((angle > self.LONG_ACCEL_THRESH) and (idx_start_of_accel != -1)):
                # end of accell
                print ("End of straight run detected " + str(idx))
                if ((idx-idx_start_of_accel) > self.LONG_ACCEL_LENGTH):
                    # it is a long run
                    print(f"    -> Long run confirmed {idx_start_of_accel}-{idx}")
                    self.tagRecords (newRecords, max(0,idx_start_of_accel-self.ANTICIPATION), idx-self.ANTICIPATION,self.LONG_RUN,1)
                    accelZoneCounter=accelZoneCounter+1
                idx_start_of_accel=-1

            if (idx_start_of_brake == -1):
                if (angle >= self.LONG_TURN_THRESH):
                    # start of turn
                    idx_start_of_brake = idx
                    print ("New turn detected @ " + str(idx_start_of_brake))

            if ((angle < self.LONG_TURN_THRESH) and (idx_start_of_brake != -1)):
                # end of turn
                print ("End of turn detected " + str(idx))
                if ((idx-idx_start_of_brake) > self.LONG_TURN_LENGTH):
                    # it is a large turn
                    print("Large turn detected @ "+str(idx_start_of_brake))
                    print(f"    -> Long turn confirmed {idx_start_of_brake}-{idx}")
                    self.tagRecords (newRecords, max(0,idx_start_of_brake-self.ANTICIPATION) , idx,self.LONG_TURN,1)
                    brakeZoneCounter=brakeZoneCounter+1
                idx_start_of_brake=-1

        for record in newRecords:
            if record[self.LONG_RUN]:
                record[self.LOCATION] = 1
            if record[self.LONG_TURN]:
                record[self.LOCATION] = 2

            tub_full.write_record(record)


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()
    tubs = args['--tubs']
    tub_path = cfg.DATA_PATH
    processor = tubPreProcessor4(tub_path, cfg)
    processor.processRecords()