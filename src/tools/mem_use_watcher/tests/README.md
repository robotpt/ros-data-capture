# Testing the 'memory_use_watcher' module and its interaction with 'video_capture' module:

## Test 1: Checking whether the watcher.py publishes the topic values correctly for 'data_capture/is_memory_usage_exceeded'

#### EXPECTED: Should publish 'False' when memory (RAM) usage is within the set threshold and 'True' when memory usage exceeds it

1. rostopic echo the 'data_capture/is_memory_usage_exceeded' topic to see that it is publishing 'False' initially when memory usage is not exceeded
2. Use the stress.py module from "https://pypi.org/project/stress/" and run it with "python stress.py -m <RAM_to_consume_in_MB>" depending on the memory of the test system to make it exceed the preset threshold percent. Once the memory utilization exceeds the threshold (can be verified from 'top'), observe that this rostopic is publishing 'True'
3. Cancel the stress.py and observe that this module is publishing 'false' once again.

## Test 2: No memory violation (Ideal case)

#### EXPECTED: Video is recorded without errors

1. Start capture.py 
2. Set the '/video_capture/is_record' topic to 'True' and observe that the video recording is started 
3. Also, rostopic echo the 'data_capture/is_memory_usage_exceeded' topic to see that it is publishing 'False'
4. Set the '/video_capture/is_record' topic to 'False' and observe that recording stopped with no errors


## Test 3: High memory utilization from the start even before recording begins

#### EXPECTED: Video recording should not start and must throw an error that it has not started due to high memory usage

1. Start capture.py
2. Manually publish 'data_capture/is_memory_usage_exceeded' topic to True
3. Set the '/video_capture/is_record' topic to 'True' 
4. Notice that video recording does not start and an error is thrown for this


## Test 4: Memory goes high in between recording (most common scenario)

#### EXPECTED: Must imediately stop recording and throw an error that recording has stopped

1. Start capture.py and set 'data_capture/is_memory_usage_exceeded' topic to False
2. Set the '/video_capture/is_record' topic to 'True' to start recording
3. Once recording has begun, set the 'data_capture/is_memory_usage_exceeded' topic to True
4. Observe that video recording is stopped immediately and an error for the same is thrown