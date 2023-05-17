# Copyright (c) 2021-2022, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.graph.core as og
import numpy as np
import random


class OgnSdTestPrintRawArray:
    @staticmethod
    def compute(db) -> bool:

        if db.state.initialSWHFrameNumber < 0:
            db.state.initialSWHFrameNumber = db.inputs.swhFrameNumber
        frameNumber = db.inputs.swhFrameNumber - db.state.initialSWHFrameNumber

        rd_seed = db.inputs.randomSeed + ((frameNumber * 17) % 491)
        random.seed(rd_seed)
        
        db.outputs.swhFrameNumber = db.inputs.swhFrameNumber
        db.outputs.exec = og.ExecutionAttributeState.ENABLED
            
        elemenType = np.uint8
        if db.inputs.elementType == db.tokens.uint16:
            elemenType = np.uint16
        elif db.inputs.elementType == db.tokens.int16:
            elemenType = np.int16
        elif db.inputs.elementType == db.tokens.uint32:
            elemenType = np.uint32
        elif db.inputs.elementType == db.tokens.int32:
            elemenType = np.int32
        elif db.inputs.elementType == db.tokens.float32:
            elemenType = np.float32
        elif db.inputs.elementType == db.tokens.token:
            elemenType = np.uint64
        
        elementCount = db.inputs.elementCount

        data = db.inputs.data
        data = data.view(elemenType)
        
        if db.inputs.mode == db.tokens.printFormatted:

            is2DArray = db.inputs.bufferSize == 0
            if not is2DArray:
                data = data.reshape(data.shape[0] // elementCount, elementCount) if elementCount > 1 else data
            else:
                data = (
                    data.reshape(db.inputs.height, db.inputs.width, elementCount)
                    if elementCount > 1
                    else data.reshape(db.inputs.height, db.inputs.width)
                )

            print("OgnSdPrintRawArray : ", db.inputs.swhFrameNumber)
            print(data)

        elif (frameNumber in db.inputs.referenceSWHFrameNumbers) and (data.shape[0]>=db.inputs.referenceNumUniqueRandomValues):
            if (db.inputs.mode == db.tokens.printReferences):
                ref_values = data.astype(np.float32)
                random.shuffle(ref_values)
                ref_values = ref_values[:db.inputs.referenceNumUniqueRandomValues]
                print(ref_values)      
            else:
                ref_values = data.astype(np.float32)
                random.shuffle(ref_values)
                ref_values = ref_values[:db.inputs.referenceNumUniqueRandomValues]
                frame_offset = np.where(db.inputs.referenceSWHFrameNumbers == frameNumber)[0][0]
                reference_offset = frame_offset * db.inputs.referenceNumUniqueRandomValues
                err = np.square(ref_values - db.inputs.referenceValues[reference_offset:reference_offset+db.inputs.referenceNumUniqueRandomValues]).max()
                if err >= db.inputs.referenceTolerance:
                    print(f"OgnSdTestPrintRawArray [Error]")

        return True
