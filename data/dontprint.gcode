;FLAVOR:Marlin
;TIME:436
;Filament used: 0.672037m
;Layer height: 0.6
;MINX:-49.582
;MINY:-49.6
;MINZ:0.3
;MAXX:49.582
;MAXY:49.6
;MAXZ:0.9
;Generated with Cura_SteamEngine 4.9.1
M104 S200
M105
M109 S200
M82 ;absolute extrusion mode
G28 ;Home
G1 Z15.0 F6000 ;Move the platform down 15mm
;Prime the extruder
G92 E0
G1 F200 E3
G92 E0
G92 E0
G92 E0
G1 F1500 E-6.5
;LAYER_COUNT:2
;LAYER:0
M107
;MESH:circles.STL
G0 F3600 X9.443 Y-1.726 Z0.3
;TYPE:WALL-OUTER
G1 F1500 E0
G1 F1800 X9.571 Y-.74 E0.03741
G1 X9.596 Y0.246 E0.07451
G1 X10.318 Y-47.381
G1 F1500 E217.51145
;MESH:NONMESH
G0 F600 X10.318 Y-47.381 Z0.9
G0 F3600 X13.818 Y-37.965
G0 X9.344 Y-1.71
G0 X9.443 Y-1.726
;TIME_ELAPSED:220.945057
;LAYER:1
M106 S255
;TYPE:WALL-OUTER
;MESH:circles.STL
G1 F1500 E224.01145
G1 F1800 X9.571 Y-.74 E224.08626
G1 X9.596 Y0.246 E224.16048
G0 X-20.46 Y-1.287
G0 X-20.361 Y-1.281
;TYPE:WALL-OUTER
G1 F1500 E253.80445
G1 F1800 X-20.361 Y1.281 E253.99722
G1 X-20.04 Y3.823 E254.19001
G0 F5400 X29.17 Y-4.915
G0 X28.34 Y-5.171
;TYPE:SKIN
G1 F1800 X28.724 Y-2.22 E277.65418
G1 X28.797 Y0.74 E277.87696
G1 X28.571 Y3.69 E278.09958
G1 X-10.317 Y-47.382
;TIME_ELAPSED:436.365394
G1 F1500 E665.53712
M107
M104 S0
M140 S0
;Retract the filament
G92 E1
G1 E-1 F300
G28 X0 Y0
M84
M82 ;absolute extrusion mode
M104 S0
;End of Gcode
;SETTING_3 {"global_quality": "[general]\\nversion = 4\\nname = Extra Coarse #2\
;SETTING_3 \ndefinition = custom\\n\\n[metadata]\\ntype = quality_changes\\nqual
;SETTING_3 ity_type = extra coarse\\nsetting_version = 16\\n\\n[values]\\nadhesi
;SETTING_3 on_type = none\\n\\n", "extruder_quality": ["[general]\\nversion = 4\
;SETTING_3 \nname = Extra Coarse #2\\ndefinition = fdmprinter\\n\\n[metadata]\\n
;SETTING_3 type = quality_changes\\nquality_type = extra coarse\\nsetting_versio
;SETTING_3 n = 16\\nposition = 0\\n\\n[values]\\n\\n"]}
