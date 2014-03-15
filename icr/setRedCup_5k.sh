#!/bin/bash

rosservice call /icr_server/set_active_phalanges '['thdistal', 'ffdistal','mfdistal','rfdistal']'
rosservice call /model_server/load_object '{file: "RedCup_5k", initial_pose: {position: [0.725, -0.08, 1.023],orientation: [0.2231,-0.6713,-0.6708,0.2229]}}'


