#!/bin/bash

rosservice call /icr_server/set_active_phalanges '['thdistal', 'ffdistal','mfdistal','rfdistal']'
rosservice call /model_server/load_object '{file: "cup", initial_pose: {position: [0.725, -0.08, 1.08],orientation: [0.0,0.0,0.9874,-0.158]}}'


