# Manual for topic_logger node 

Tool for recording messages sent between nodes with ability to filter/ignore selected topics. 

## Usage

1. import LoggerContol from drone_hardware
2. start logging (preferably at the begining of the mission) via start() function from LoggerControl class
3. stop logging via stop() function

## Filtering

Defined inside IgnoredTopics class in topic_logger.py. topics_to_ignore variable handles list of topics with 0/1 values indicating if a topic is ignored or not (0 - tracked, 1 - ignored). 

Example: 

```python
{"/fmu/out/vehicle_global_position" : 1,
 "/fmu/in/arming_check_reply_v1":     0,}
```

in this case, messages from first topic will be untracked, while those from second one - tracked

## How it works

1. start() function creates separate thread on which topic_logger node runs to ensure continuous log
2. creates topics_logs/ directiory (if nonexistent) inside drone_hardware
3. creates topic_log_n.csv file, where n is log number (if no logs -> defaults to 1)
4. gives log file header with date+time of file creation
5. finds all active topics, discards ones listed in IgnoredTopics and subscribes to the rest (if topic not listed - treats as tracked)
6. logs messages formatted as: time; topic; message
7. stop() funtion closes log file, destroys node and kills thread on which it run