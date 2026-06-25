# Skill for debug sine wave last joint

Use this skill to debug if Crospi and your (real) robot are being interfaced correctly.

By default it only works for a 6DOF robot. To change it to less/more DOFs, edit the `skill_for_debug_sine_wave_last_joint.json` and change the number of elements in the ampitude and frequency vector. 

E.g. for 7 DOF:

```json
{
    "$schema": "../../../../task_specifications/tasks-schema.json",
    "tasks": [
        {
            "name": "SineWaveLastJoint",
            "task_specification":{
                "from-debug_lib-version-0.1.0": true,
                "is-move_sine_waves_jointspace": true,
                "file_path": "$[crospi_application_template]/task_specifications/libraries/debug_lib/task_specifications/move_sine_waves_jointspace.etasl.lua",
                "parameters": {
                    "amplitudes": [
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0.35
                    ],
                    "frequencies": [
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0.2
                    ],
                    "execution_time": 5
                }
            }
        }
    ]
}
```