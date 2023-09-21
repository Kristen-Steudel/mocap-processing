*** This data was generated with AddBiomechanics (www.addbiomechanics.org) ***
AddBiomechanics was written by Keenon Werling.

Automatic processing achieved the following marker errors (averaged
over all frames of all trials):

- Avg. Marker RMSE      = 1.80 cm
- Avg. Max Marker Error = 7.21 cm

Automatic processing reduced the residual loads needed for dynamic
consistency to the following magnitudes (averaged over all frames of
all trials):

- Avg. Residual Force  = 0.92 N
- Avg. Residual Torque = 22.89 N-m

Automatic processing found a new model mass to achieve dynamic
consistency:

  - Total mass = 65.92 kg (-0.45% change from original 66.22 kg)

Individual body mass changes:

  - pelvis    mass = 9.47 kg (-8.69% change from original 10.38 kg)
  - femur_r   mass = 7.64 kg (-6.79% change from original 8.19 kg)
  - tibia_r   mass = 2.61 kg (-20.01% change from original 3.27 kg)
  - talus_r   mass = 0.08 kg (-7.91% change from original 0.09 kg)
  - calcn_r   mass = 0.45 kg (-59.47% change from original 1.10 kg)
  - toes_r    mass = 0.18 kg (-3.73% change from original 0.19 kg)
  - femur_l   mass = 7.64 kg (-6.79% change from original 8.19 kg)
  - tibia_l   mass = 2.61 kg (-20.01% change from original 3.27 kg)
  - talus_l   mass = 0.08 kg (-7.91% change from original 0.09 kg)
  - calcn_l   mass = 0.45 kg (-59.47% change from original 1.10 kg)
  - toes_l    mass = 0.18 kg (-3.73% change from original 0.19 kg)
  - torso     mass = 29.74 kg (+25.85% change from original 23.63 kg)
  - humerus_r mass = 1.19 kg (-33.33% change from original 1.79 kg)
  - ulna_r    mass = 0.48 kg (-10.37% change from original 0.54 kg)
  - radius_r  mass = 0.48 kg (-10.37% change from original 0.54 kg)
  - hand_r    mass = 0.24 kg (-41.18% change from original 0.40 kg)
  - humerus_l mass = 1.19 kg (-33.33% change from original 1.79 kg)
  - ulna_l    mass = 0.48 kg (-10.37% change from original 0.54 kg)
  - radius_l  mass = 0.48 kg (-10.37% change from original 0.54 kg)
  - hand_l    mass = 0.24 kg (-41.18% change from original 0.40 kg)

The following trials were processed to perform automatic body scaling,
marker registration, and residual reduction:

trial: Trimmed_NordZeroed000004
  - Avg. Marker RMSE      = 1.80 cm
  - Avg. Marker Max Error = 7.21 cm
  - Avg. Residual Force   = 0.92 N
  - Avg. Residual Torque  = 22.89 N-m
  - WARNING: 3 marker(s) with RMSE greater than 4 cm!
  - WARNING: Automatic data processing required modifying TRC data from 3 marker(s)!
  --> See IK/Trimmed_NordZeroed000004_ik_summary.txt and ID/Trimmed_NordZeroed000004_id_summary.txt for more details.


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

 > opensim-cmd run-tool Trimmed_NordZeroed000004_ik_setup.xml
           # This will create a results file IK/Trimmed_NordZeroed000004_ik_by_opensim.mot


To re-run Inverse Dynamics using OpenSim, you can also use
automatically generated XML configuration files. WARNING: Inverse
Dynamics in OpenSim uses a different time-step definition to the one
used in AddBiomechanics (AddBiomechanics uses semi-implicit Euler,
OpenSim uses splines). This means that your OpenSim inverse dynamics
results WILL NOT MATCH your AddBiomechanics results, and YOU SHOULD
NOT EXPECT THEM TO. The following commands should work (FROM THE "ID"
FOLDER, and not including the leading "> "):

 > opensim-cmd run-tool Trimmed_NordZeroed000004_id_setup.xml
           # This will create results on time range (0.005s to 9.54s) in file ID/Trimmed_NordZeroed000004_osim_id.sto


The original unscaled model file is present in:

Models/unscaled_generic.osim

There is also an unscaled model, with markers moved to spots found by
this tool, at:

Models/unscaled_but_with_optimized_markers.osim

If you encounter errors, please submit a post to the AddBiomechanics
user forum on SimTK.org:

   https://simtk.org/projects/addbiomechanics