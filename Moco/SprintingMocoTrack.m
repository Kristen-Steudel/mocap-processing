% Kristen Steudel September 13, 2023

function SprintingMocoTrack()
import org.opensim.modeling.*;

%Define a tracking problem
track = MocoTrack();
track.setName("muscle_driven_state_tracking");

addpath("S001_0450\")

% Put in the scaled model here
modelProcessor = ModelProcessor("final.osim");
% Figure out what this xml document needs to contain, open it up in the
% example
% Include the grf file that this xml points to in the working folder
modelProcessor.append(ModOpAddExternalLoads("rotated_Sprint_04500001_external_forces_raw.xml")); % Ground reaction forces xml document
% Only valid with DeGrooteFregly2016Muscles
modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
track.setModel(modelProcessor);

%Construct a table processor of the coordinate data and pass it to the
%tracking tool. Table processors can be used in the same way as
%ModelProcessors by appending TableOperators to modify the base table. A
%TableProcessor with no operators, as we have here, simply returns the base
%table.
% Do I load in the trc file or convert the trc file to an sto file for the
% "coordinates.sto"? Put in the inverse kinematics results here
tableProcessor = TableProcessor("rotated_Sprint_04500001_ik.mot");
% May need to use this command with a .mot file
tableProcessor.append(TabOpUseAbsoluteStateNames());

track.setStatesReference(tableProcessor)
track.set_states_global_tracking_weight(10);
% This setting allows extra data columns contained in the states reference
% that don't correspond to model coordinates.
track.set_allow_unused_references(true);
% Since there is only coordinate position data in the states references,
% this setting is enabled to fill in the missing coordinate speed data
% using the derivative of the splined position data
track.set_track_reference_position_derivatives(true);

%Initial time, final time, and mesh interval
track.set_initial_time(0.05);
track.set_final_time(0.55);
track.set_mesh_interval(0.05);

% Instead of calling solve(), call initialize() to receive a preconfigured
% MocoStudy object based on the settings above. Use this to customize the
% problem beyond the MocoTrack interface.
% solution = track.solve();
% solution.write('muscledrivenMocoTrack_solution.sto')

study = track.initialize();

%get a reference to the MocoControl Goal that is added to every MocoTrack
%problem by default
problem = study.updProblem();

effort = MocoControlGoal.safeDownCast(problem.updGoal("control_effort"));

%Put a large weight on the pelvis coordinate actuators, which act as the
%residual
model = modelProcessor.process();
model.initSystem();
forceSet = model.getForceSet();
for i = 0:forceSet.getSize()-1
    forcePath = forceSet.get(i).getAbsolutePathString();
    if contains(string(forcePath),'pelvis')
        effort.setWeightforControl(forcePath,10);
    end
end

% Need to configure solver ?
% % Configure the solver.
% % =====================
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(20);
solver.set_optim_constraint_tolerance(1e-3);
solver.set_optim_convergence_tolerance(1e-3);

%Solve and visualize
solution = study.solve();
study.visualize(solution);
solution.write('muscledrivenMocoTrack_solution.sto')

% Generate a PDF report containing plots of the variables in the solution.
% For details, see osimMocoTrajectoryReport.m in Moco's
% Resources/Code/Matlab/Utilities folder.
model = modelProcessor.process();
report = osimMocoTrajectoryReport(model, ...
            'muscledrivenMocoTrack_solution.sto');
reportFilepath = report.generate();
open(reportFilepath);


end
