*** This data was generated with AddBiomechanics (www.addbiomechanics.org) ***
AddBiomechanics was written by Keenon Werling.

Automatic processing achieved the following marker errors (averaged
over all frames of all trials):

- Avg. Marker RMSE      = 4.07 cm
- Avg. Max Marker Error = 12.90 cm

Automatic processing reduced the residual loads needed for dynamic
consistency to the following magnitudes (averaged over all frames of
all trials):

- Avg. Residual Force  = 165.37 N
- Avg. Residual Torque = 184.52 N-m

Automatic processing found a new model mass to achieve dynamic
consistency:

  - Total mass = 92.93 kg (-2.18% change from original 95.0 kg)

Individual body mass changes:

  - pelvis    mass = 19.14 kg (+28.60% change from original 14.88 kg)
  - femur_r   mass = 3.07 kg (-73.88% change from original 11.76 kg)
  - tibia_r   mass = 5.19 kg (+10.73% change from original 4.69 kg)
  - talus_r   mass = 1.50 kg (+1087.58% change from original 0.13 kg)
  - calcn_r   mass = 0.86 kg (-45.53% change from original 1.58 kg)
  - toes_r    mass = 0.20 kg (-26.02% change from original 0.27 kg)
  - femur_l   mass = 3.07 kg (-73.88% change from original 11.76 kg)
  - tibia_l   mass = 5.19 kg (+10.73% change from original 4.69 kg)
  - talus_l   mass = 1.50 kg (+1087.58% change from original 0.13 kg)
  - calcn_l   mass = 0.86 kg (-45.53% change from original 1.58 kg)
  - toes_l    mass = 0.20 kg (-26.02% change from original 0.27 kg)
  - torso     mass = 34.83 kg (+2.73% change from original 33.91 kg)
  - humerus_r mass = 1.95 kg (-24.11% change from original 2.57 kg)
  - ulna_r    mass = 1.19 kg (+54.86% change from original 0.77 kg)
  - radius_r  mass = 1.19 kg (+54.86% change from original 0.77 kg)
  - hand_r    mass = 4.33 kg (+648.55% change from original 0.58 kg)
  - humerus_l mass = 1.95 kg (-24.11% change from original 2.57 kg)
  - ulna_l    mass = 1.19 kg (+54.86% change from original 0.77 kg)
  - radius_l  mass = 1.19 kg (+54.86% change from original 0.77 kg)
  - hand_l    mass = 4.33 kg (+648.55% change from original 0.58 kg)

The following trials were processed to perform automatic body scaling,
marker registration, and residual reduction:

trial: rotated_Sprint_04500001
  - Avg. Marker RMSE      = 2.25 cm
  - Avg. Marker Max Error = 5.28 cm
  - Avg. Residual Force   = 13.62 N
  - Avg. Residual Torque  = 111.14 N-m
  - WARNING: 1 marker(s) with RMSE greater than 4 cm!
  - WARNING: Automatic data processing required modifying TRC data from 3 marker(s)!
  --> See IK/rotated_Sprint_04500001_ik_summary.txt and ID/rotated_Sprint_04500001_id_summary.txt for more details.

trial: rotated_Sprint_05400001
  - Avg. Marker RMSE      = 2.32 cm
  - Avg. Marker Max Error = 5.35 cm
  - Avg. Residual Force   = 18.72 N
  - Avg. Residual Torque  = 129.35 N-m
  - WARNING: 2 marker(s) with RMSE greater than 4 cm!
  - WARNING: Automatic data processing required modifying TRC data from 8 marker(s)!
  --> See IK/rotated_Sprint_05400001_ik_summary.txt and ID/rotated_Sprint_05400001_id_summary.txt for more details.

trial: rotated_Sprint_06300001
  - Avg. Marker RMSE      = 2.50 cm
  - Avg. Marker Max Error = 6.29 cm
  - Avg. Residual Force   = 17.66 N
  - Avg. Residual Torque  = 164.77 N-m
  - WARNING: 3 marker(s) with RMSE greater than 4 cm!
  - WARNING: Automatic data processing required modifying TRC data from 8 marker(s)!
  --> See IK/rotated_Sprint_06300001_ik_summary.txt and ID/rotated_Sprint_06300001_id_summary.txt for more details.

trial: rotated_Sprint_07200001
  - Avg. Marker RMSE      = 3.28 cm
  - Avg. Marker Max Error = 10.70 cm
  - Avg. Residual Force   = 29.44 N
  - Avg. Residual Torque  = 193.54 N-m
  - WARNING: 6 marker(s) with RMSE greater than 4 cm!
  - WARNING: Automatic data processing required modifying TRC data from 73 marker(s)!
  --> See IK/rotated_Sprint_07200001_ik_summary.txt and ID/rotated_Sprint_07200001_id_summary.txt for more details.

trial: rotated_Sprint_08100001
  - Avg. Marker RMSE      = 7.80 cm
  - Avg. Marker Max Error = 28.00 cm
  - Avg. Residual Force   = 575.52 N
  - Avg. Residual Torque  = 273.20 N-m
  - WARNING: 6 joints hit joint limits!
  - WARNING: 17 marker(s) with RMSE greater than 4 cm!
  - WARNING: Automatic data processing required modifying TRC data from 9 marker(s)!
  --> See IK/rotated_Sprint_08100001_ik_summary.txt and ID/rotated_Sprint_08100001_id_summary.txt for more details.

trial: rotated_Sprint_08550001
  - Avg. Marker RMSE      = 4.02 cm
  - Avg. Marker Max Error = 12.74 cm
  - Avg. Residual Force   = 34.37 N
  - Avg. Residual Torque  = 189.01 N-m
  - WARNING: 6 marker(s) with RMSE greater than 4 cm!
  - WARNING: Automatic data processing required modifying TRC data from 18 marker(s)!
  --> See IK/rotated_Sprint_08550001_ik_summary.txt and ID/rotated_Sprint_08550001_id_summary.txt for more details.


The model file containing optimal body scaling, marker offsets, and
mass parameters is:

Models/final.osim

This tool works by finding optimal scale factors and marker offsets at
the same time. If specified, it also runs a second optimization to
find mass parameters to fit the model dynamics to the ground reaction
force data.

The model containing the optimal body scaling and marker offsets found
prior to the dynamics fitting step is:

Models/optimized_scale_and_markers.osim

If you want to manually edit the marker offsets, you can modify the
<MarkerSet> in "Models/unscaled_but_with_optimized_markers.osim" (by
default this file contains the marker offsets found by the optimizer).
If you want to tweak the Scaling, you can edit
"Models/rescaling_setup.xml". If you change either of these files,
then run (FROM THE "Models" FOLDER, and not including the leading ">
"):

 > opensim-cmd run-tool rescaling_setup.xml
           # This will re-generate Models/optimized_scale_and_markers.osim


You do not need to re-run Inverse Kinematics unless you change
scaling, because the output motion files are already generated for you
as "*_ik.mot" files for each trial, but you are welcome to confirm our
results using OpenSim. To re-run Inverse Kinematics with OpenSim, to
verify the results of AddBiomechanics, you can use the automatically
generated XML configuration files. Here are the command-line commands
you can run (FROM THE "IK" FOLDER, and not including the leading "> ")
to verify IK results for each trial:

 > opensim-cmd run-tool rotated_Sprint_04500001_ik_setup.xml
           # This will create a results file IK/rotated_Sprint_04500001_ik_by_opensim.mot
 > opensim-cmd run-tool rotated_Sprint_05400001_ik_setup.xml
           # This will create a results file IK/rotated_Sprint_05400001_ik_by_opensim.mot
 > opensim-cmd run-tool rotated_Sprint_06300001_ik_setup.xml
           # This will create a results file IK/rotated_Sprint_06300001_ik_by_opensim.mot
 > opensim-cmd run-tool rotated_Sprint_07200001_ik_setup.xml
           # This will create a results file IK/rotated_Sprint_07200001_ik_by_opensim.mot
 > opensim-cmd run-tool rotated_Sprint_08100001_ik_setup.xml
           # This will create a results file IK/rotated_Sprint_08100001_ik_by_opensim.mot
 > opensim-cmd run-tool rotated_Sprint_08550001_ik_setup.xml
           # This will create a results file IK/rotated_Sprint_08550001_ik_by_opensim.mot


To re-run Inverse Dynamics using OpenSim, you can also use
automatically generated XML configuration files. WARNING: Inverse
Dynamics in OpenSim uses a different time-step definition to the one
used in AddBiomechanics (AddBiomechanics uses semi-implicit Euler,
OpenSim uses splines). This means that your OpenSim inverse dynamics
results WILL NOT MATCH your AddBiomechanics results, and YOU SHOULD
NOT EXPECT THEM TO. The following commands should work (FROM THE "ID"
FOLDER, and not including the leading "> "):

 > opensim-cmd run-tool rotated_Sprint_04500001_id_setup.xml
           # This will create results on time range (0.005s to 3.915s) in file ID/rotated_Sprint_04500001_osim_id.sto
 > opensim-cmd run-tool rotated_Sprint_05400001_id_setup.xml
           # This will create results on time range (0.005s to 3.61s) in file ID/rotated_Sprint_05400001_osim_id.sto
 > opensim-cmd run-tool rotated_Sprint_06300001_id_setup.xml
           # This will create results on time range (0.005s to 3.58s) in file ID/rotated_Sprint_06300001_osim_id.sto
 > opensim-cmd run-tool rotated_Sprint_07200001_id_setup.xml
           # This will create results on time range (0.005s to 3.17s) in file ID/rotated_Sprint_07200001_osim_id.sto
 > opensim-cmd run-tool rotated_Sprint_08100001_id_setup.xml
           # This will create results on time range (0.005s to 5.845s) in file ID/rotated_Sprint_08100001_osim_id.sto
 > opensim-cmd run-tool rotated_Sprint_08550001_id_setup.xml
           # This will create results on time range (0.01s to 2.38s) in file ID/rotated_Sprint_08550001_osim_id.sto


The original unscaled model file is present in:

Models/unscaled_generic.osim

There is also an unscaled model, with markers moved to spots found by
this tool, at:

Models/unscaled_but_with_optimized_markers.osim

If you encounter errors, please submit a post to the AddBiomechanics
user forum on SimTK.org:

   https://simtk.org/projects/addbiomechanics