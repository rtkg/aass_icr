#!/bin/bash

rosservice call /icr_server/set_active_phalanges '['thdistal', 'ffdistal','mfdistal','rfdistal']'
rosservice call /model_server/load_object '{file: "CokePlasticSmall_5k", initial_pose: {position: [0.82, -0.055, 1.14],orientation: [0.2888,0.6454,0.6454,0.2888]}}'


