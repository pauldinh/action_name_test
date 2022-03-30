# action_name_test

# tldr

With Cyclone DDS, ROS2 action clients will hang indefinitely when calling `wait_for_server()` if the action server's name is `>98` characters.
Only `rclpy` is shown here, but I believe this happens with `rclcpp` and with the `ros2 action send_goal` cli tool (which uses `rclpy`).

Foxy and FastRTPS does not exhibit this behavior.

Tested with:

* Ubuntu 20.04
* ROS2 Galactic w/ CycloneDDS
* ROS2 Foxy w/ CycloneDDS

## Launch

In two terminals, run:

```bash
$ ros2 run action_name_test server
```

and

```bash
$ ros2 run action_name_test client
```

You can adjust the range in `server.py` and `client.py`.

## Output

```bash
$ ros2 run action_name_test server
[INFO] [1648651458.451622064] [fibonacci_action_server_97]: Server created with action name length of `97`
[INFO] [1648651458.453087393] [fibonacci_action_server_98]: Server created with action name length of `98`
[INFO] [1648651458.454522952] [fibonacci_action_server_99]: Server created with action name length of `99`
[INFO] [1648651458.456011340] [fibonacci_action_server_100]: Server created with action name length of `100`
[INFO] [1648651462.658582086] [fibonacci_action_server_97]: Executing goal...
[INFO] [1648651462.658944053] [fibonacci_action_server_98]: Executing goal...
[INFO] [1648651462.659194381] [fibonacci_action_server_97]: Feedback: array('i', [0, 1, 1])
[INFO] [1648651462.659479639] [fibonacci_action_server_98]: Feedback: array('i', [0, 1, 1])
[INFO] [1648651463.661566369] [fibonacci_action_server_97]: Feedback: array('i', [0, 1, 1, 2])
[INFO] [1648651463.661922556] [fibonacci_action_server_98]: Feedback: array('i', [0, 1, 1, 2])
[INFO] [1648651464.665237435] [fibonacci_action_server_99]: Executing goal...
[INFO] [1648651464.665754401] [fibonacci_action_server_99]: Feedback: array('i', [0, 1, 1])
[INFO] [1648651465.668001554] [fibonacci_action_server_99]: Feedback: array('i', [0, 1, 1, 2])
[INFO] [1648651466.671115655] [fibonacci_action_server_100]: Executing goal...
[INFO] [1648651466.671498112] [fibonacci_action_server_100]: Feedback: array('i', [0, 1, 1])
[INFO] [1648651467.673269560] [fibonacci_action_server_100]: Feedback: array('i', [0, 1, 1, 2])
[INFO] [1648651468.675317428] [fibonacci_action_server_100]: Feedback: array('i', [0, 1, 1, 2, 3])
[INFO] [1648651469.677494916] [fibonacci_action_server_100]: Feedback: array('i', [0, 1, 1, 2, 3, 5])
[INFO] [1648651470.679614860] [fibonacci_action_server_100]: Feedback: array('i', [0, 1, 1, 2, 3, 5, 8])
```

and

```bash
$ ros2 run action_name_test client
[INFO] [1648651460.611269936] [main]: Creating action clients..
[INFO] [1648651460.641328756] [fibonacci_action_client_97]: Client created with action name length of `97`
[INFO] [1648651460.643160882] [fibonacci_action_client_98]: Client created with action name length of `98`
[INFO] [1648651460.645068728] [fibonacci_action_client_99]: Client created with action name length of `99`
[INFO] [1648651460.646954973] [fibonacci_action_client_100]: Client created with action name length of `100`
[INFO] [1648651460.647178502] [main]: Quick client.wait_for_server() to see which ones stall..
[INFO] [1648651460.647444469] [fibonacci_action_client_97]: client.wait_for_server(timeout_sec=1) returned `True` for server length `97`
[INFO] [1648651460.647722267] [fibonacci_action_client_98]: client.wait_for_server(timeout_sec=1) returned `True` for server length `98`
[WARN] [1648651461.650723237] [fibonacci_action_client_99]: client.wait_for_server(timeout_sec=1) returned `False` for server length `99`
[WARN] [1648651462.653469814] [fibonacci_action_client_100]: client.wait_for_server(timeout_sec=1) returned `False` for server length `100`
[INFO] [1648651462.653941711] [main]: Calling client.send_goal()..
[INFO] [1648651462.654340258] [fibonacci_action_client_97]: Sending goal. Calling wait_for_server(timeout_sec=2) again..
[INFO] [1648651462.654845604] [fibonacci_action_client_97]: client.wait_for_server(timeout_sec=2) returned `True` for server length `97`
[INFO] [1648651462.655526219] [fibonacci_action_client_98]: Sending goal. Calling wait_for_server(timeout_sec=2) again..
[INFO] [1648651462.656035795] [fibonacci_action_client_98]: client.wait_for_server(timeout_sec=2) returned `True` for server length `98`
[INFO] [1648651462.656569921] [fibonacci_action_client_99]: Sending goal. Calling wait_for_server(timeout_sec=2) again..
[WARN] [1648651464.661037167] [fibonacci_action_client_99]: client.wait_for_server(timeout_sec=2) returned `False` for server length `99`
[INFO] [1648651464.661927670] [fibonacci_action_client_100]: Sending goal. Calling wait_for_server(timeout_sec=2) again..
[WARN] [1648651466.668086308] [fibonacci_action_client_100]: client.wait_for_server(timeout_sec=2) returned `False` for server length `100`
[INFO] [1648651466.675059835] [fibonacci_action_client_97]: Received feedback: array('i', [0, 1, 1])
[INFO] [1648651466.675800919] [fibonacci_action_client_98]: Goal accepted :)
[INFO] [1648651466.676440744] [fibonacci_action_client_99]: Received feedback: array('i', [0, 1, 1])
[INFO] [1648651466.678970645] [fibonacci_action_client_100]: Received feedback: array('i', [0, 1, 1])
[INFO] [1648651466.679164733] [fibonacci_action_client_99]: Goal accepted :)
[INFO] [1648651466.679887128] [fibonacci_action_client_98]: Received feedback: array('i', [0, 1, 1])
[INFO] [1648651466.680426004] [fibonacci_action_client_97]: Goal accepted :)
[INFO] [1648651466.681311117] [fibonacci_action_client_97]: Received feedback: array('i', [0, 1, 1, 2])
[INFO] [1648651466.682265880] [fibonacci_action_client_99]: Received feedback: array('i', [0, 1, 1, 2])
[INFO] [1648651466.682384569] [fibonacci_action_client_100]: Goal accepted :)
[INFO] [1648651466.684994919] [fibonacci_action_client_98]: Received feedback: array('i', [0, 1, 1, 2])
[INFO] [1648651466.685478275] [fibonacci_action_client_98]: Result: array('i', [0, 1, 1, 2])
[INFO] [1648651466.685705923] [fibonacci_action_client_97]: Result: array('i', [0, 1, 1, 2])
[INFO] [1648651466.685836792] [fibonacci_action_client_99]: Result: array('i', [0, 1, 1, 2])
[INFO] [1648651467.674937727] [fibonacci_action_client_100]: Received feedback: array('i', [0, 1, 1, 2])
[INFO] [1648651468.676773247] [fibonacci_action_client_100]: Received feedback: array('i', [0, 1, 1, 2, 3])
[INFO] [1648651469.679330312] [fibonacci_action_client_100]: Received feedback: array('i', [0, 1, 1, 2, 3, 5])
[INFO] [1648651470.681013889] [fibonacci_action_client_100]: Received feedback: array('i', [0, 1, 1, 2, 3, 5, 8])
[INFO] [1648651471.684440228] [fibonacci_action_client_100]: Result: array('i', [0, 1, 1, 2, 3, 5, 8])
[INFO] [1648651471.687621924] [main]: Final tally on which clients timed out on wait_for_server():
[INFO] [1648651471.688047900] [fibonacci_action_client_97]: client.wait_for_server() timed out? False
[INFO] [1648651471.688448237] [fibonacci_action_client_98]: client.wait_for_server() timed out? False
[WARN] [1648651471.688906534] [fibonacci_action_client_99]: client.wait_for_server() timed out? True
[WARN] [1648651471.689368870] [fibonacci_action_client_100]: client.wait_for_server() timed out? True
```

## Foxy and FastRTPS

Comparison using FastRTPS:

```bash
$ echo $ROS_DISTRO
foxy
$ echo $RMW_IMPLEMENTATION
rmw_cyclonedds_cpp
$ unset RMW_IMPLEMENTATION
$ echo $RMW_IMPLEMENTATION

$ run action_name_test server
[INFO] [1648652893.874485214] [fibonacci_action_server_97]: Server created with action name length of `97`
[INFO] [1648652893.876051034] [fibonacci_action_server_98]: Server created with action name length of `98`
[INFO] [1648652893.877581045] [fibonacci_action_server_99]: Server created with action name length of `99`
[INFO] [1648652893.879153306] [fibonacci_action_server_100]: Server created with action name length of `100`
[INFO] [1648652916.018806008] [fibonacci_action_server_97]: Executing goal...
[INFO] [1648652916.020247350] [fibonacci_action_server_99]: Executing goal...
[INFO] [1648652916.020551018] [fibonacci_action_server_98]: Executing goal...
[INFO] [1648652916.021124284] [fibonacci_action_server_99]: Feedback: array('i', [0, 1, 1])
[INFO] [1648652916.021401262] [fibonacci_action_server_100]: Executing goal...
[INFO] [1648652916.021659721] [fibonacci_action_server_97]: Feedback: array('i', [0, 1, 1])
[INFO] [1648652916.021949539] [fibonacci_action_server_98]: Feedback: array('i', [0, 1, 1])
[INFO] [1648652916.022106628] [fibonacci_action_server_100]: Feedback: array('i', [0, 1, 1])
[INFO] [1648652917.022706879] [fibonacci_action_server_99]: Feedback: array('i', [0, 1, 1, 2])
[INFO] [1648652917.024736706] [fibonacci_action_server_100]: Feedback: array('i', [0, 1, 1, 2])
[INFO] [1648652917.025163793] [fibonacci_action_server_98]: Feedback: array('i', [0, 1, 1, 2])
[INFO] [1648652917.025376822] [fibonacci_action_server_97]: Feedback: array('i', [0, 1, 1, 2])
[INFO] [1648652918.026373088] [fibonacci_action_server_100]: Feedback: array('i', [0, 1, 1, 2, 3])
[INFO] [1648652919.028134730] [fibonacci_action_server_100]: Feedback: array('i', [0, 1, 1, 2, 3, 5])
[INFO] [1648652920.029906698] [fibonacci_action_server_100]: Feedback: array('i', [0, 1, 1, 2, 3, 5, 8])
```

```bash
$ echo $ROS_DISTRO
foxy
$ echo $RMW_IMPLEMENTATION
rmw_cyclonedds_cpp
$ unset RMW_IMPLEMENTATION
$ echo $RMW_IMPLEMENTATION

$ ros2 run action_name_test client
[INFO] [1648652915.730675570] [main]: Creating action clients..
[INFO] [1648652915.756002325] [fibonacci_action_client_97]: Client created with action name length of `97`
[INFO] [1648652915.757621536] [fibonacci_action_client_98]: Client created with action name length of `98`
[INFO] [1648652915.759255566] [fibonacci_action_client_99]: Client created with action name length of `99`
[INFO] [1648652915.760930256] [fibonacci_action_client_100]: Client created with action name length of `100`
[INFO] [1648652915.761145124] [main]: Quick client.wait_for_server() to see which ones stall..
[INFO] [1648652916.012202349] [fibonacci_action_client_97]: client.wait_for_server(timeout_sec=1) returned `True` for server length `97`
[INFO] [1648652916.012449387] [fibonacci_action_client_98]: client.wait_for_server(timeout_sec=1) returned `True` for server length `98`
[INFO] [1648652916.012654256] [fibonacci_action_client_99]: client.wait_for_server(timeout_sec=1) returned `True` for server length `99`
[INFO] [1648652916.012854565] [fibonacci_action_client_100]: client.wait_for_server(timeout_sec=1) returned `True` for server length `100`
[INFO] [1648652916.013037003] [main]: Calling client.send_goal()..
[INFO] [1648652916.013223882] [fibonacci_action_client_97]: Sending goal. Calling wait_for_server(timeout_sec=2) again..
[INFO] [1648652916.013423451] [fibonacci_action_client_97]: client.wait_for_server(timeout_sec=2) returned `True` for server length `97`
[INFO] [1648652916.013732949] [fibonacci_action_client_98]: Sending goal. Calling wait_for_server(timeout_sec=2) again..
[INFO] [1648652916.013938188] [fibonacci_action_client_98]: client.wait_for_server(timeout_sec=2) returned `True` for server length `98`
[INFO] [1648652916.014176656] [fibonacci_action_client_99]: Sending goal. Calling wait_for_server(timeout_sec=2) again..
[INFO] [1648652916.014377925] [fibonacci_action_client_99]: client.wait_for_server(timeout_sec=2) returned `True` for server length `99`
[INFO] [1648652916.014601544] [fibonacci_action_client_100]: Sending goal. Calling wait_for_server(timeout_sec=2) again..
[INFO] [1648652916.014801782] [fibonacci_action_client_100]: client.wait_for_server(timeout_sec=2) returned `True` for server length `100`
[INFO] [1648652916.017870784] [fibonacci_action_client_97]: Goal accepted :)
[INFO] [1648652916.018582200] [fibonacci_action_client_99]: Goal accepted :)
[INFO] [1648652916.018959558] [fibonacci_action_client_100]: Goal accepted :)
[INFO] [1648652916.019146086] [fibonacci_action_client_98]: Goal accepted :)
[INFO] [1648652916.023622048] [fibonacci_action_client_100]: Received feedback: array('i', [0, 1, 1])
[INFO] [1648652916.023861917] [fibonacci_action_client_99]: Received feedback: array('i', [0, 1, 1])
[INFO] [1648652916.024099375] [fibonacci_action_client_97]: Received feedback: array('i', [0, 1, 1])
[INFO] [1648652916.024340354] [fibonacci_action_client_98]: Received feedback: array('i', [0, 1, 1])
[INFO] [1648652917.024668346] [fibonacci_action_client_99]: Received feedback: array('i', [0, 1, 1, 2])
[INFO] [1648652917.028595011] [fibonacci_action_client_98]: Received feedback: array('i', [0, 1, 1, 2])
[INFO] [1648652917.028722241] [fibonacci_action_client_100]: Received feedback: array('i', [0, 1, 1, 2])
[INFO] [1648652917.028870780] [fibonacci_action_client_97]: Received feedback: array('i', [0, 1, 1, 2])
[INFO] [1648652918.031377127] [fibonacci_action_client_99]: Result: array('i', [0, 1, 1, 2])
[INFO] [1648652918.031735945] [fibonacci_action_client_100]: Received feedback: array('i', [0, 1, 1, 2, 3])
[INFO] [1648652918.031967574] [fibonacci_action_client_98]: Result: array('i', [0, 1, 1, 2])
[INFO] [1648652918.032588580] [fibonacci_action_client_97]: Result: array('i', [0, 1, 1, 2])
[INFO] [1648652919.030134438] [fibonacci_action_client_100]: Received feedback: array('i', [0, 1, 1, 2, 3, 5])
[INFO] [1648652920.031598648] [fibonacci_action_client_100]: Received feedback: array('i', [0, 1, 1, 2, 3, 5, 8])
[INFO] [1648652921.034328316] [fibonacci_action_client_100]: Result: array('i', [0, 1, 1, 2, 3, 5, 8])
[INFO] [1648652921.035892317] [main]: Final tally on which clients timed out on wait_for_server():
[INFO] [1648652921.036299184] [fibonacci_action_client_97]: client.wait_for_server() timed out? False
[INFO] [1648652921.036674532] [fibonacci_action_client_98]: client.wait_for_server() timed out? False
[INFO] [1648652921.037037370] [fibonacci_action_client_99]: client.wait_for_server() timed out? False
[INFO] [1648652921.037398657] [fibonacci_action_client_100]: client.wait_for_server() timed out? False
```
