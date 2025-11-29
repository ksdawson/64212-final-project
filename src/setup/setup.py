from pydrake.all import (
    DiagramBuilder
)
from manipulation.station import LoadScenario, MakeHardwareStation, AddPointClouds, MakeMultibodyPlant
from setup.setup_simulation import get_scenario
from motion.motion import TrajectoryController
from grasp.grasp import GraspController
from setup.setup_simulation import get_scenario

def setup_station(meshcat=None):
    # Get scenario
    scenario_string = get_scenario()

    # Load the scenario and build the simulation station
    scenario = LoadScenario(data=scenario_string)
    if meshcat is None:
        station = MakeHardwareStation(scenario)
    else:
        station = MakeHardwareStation(scenario, meshcat=meshcat)

    # Build a Drake Diagram containing the station
    builder = DiagramBuilder()
    builder.AddSystem(station)

    return builder, station, scenario

def setup(meshcat=None):
    # Setup the station
    builder, station, scenario = setup_station(meshcat)

    # Setup getting point clouds from the cameras
    to_point_cloud = AddPointClouds(
        scenario=scenario, station=station, builder=builder # can render w/ meshcat but much slower
    )
    builder.ExportOutput(to_point_cloud['camera0'].get_output_port(), 'camera0_point_cloud')
    builder.ExportOutput(to_point_cloud['camera1'].get_output_port(), 'camera1_point_cloud')
    builder.ExportOutput(to_point_cloud['camera2'].get_output_port(), 'camera2_point_cloud')

    # Make a plant including only the iiwa and gripper for the controllers
    plant_iiwa1 = MakeMultibodyPlant(
        scenario=scenario, model_instance_names=['iiwa1', 'wsg1']
    )
    plant_iiwa2 = MakeMultibodyPlant(
        scenario=scenario, model_instance_names=['iiwa2', 'wsg2']
    )

    # Add controllers
    iiwa1_traj_controller = builder.AddNamedSystem('iiwa1_traj_controller', TrajectoryController(plant_iiwa1))
    iiwa2_traj_controller = builder.AddNamedSystem('iiwa2_traj_controller', TrajectoryController(plant_iiwa2))
    iiwa1_grasp_controller = builder.AddNamedSystem('iiwa1_grasp_controller', GraspController(plant_iiwa1))
    iiwa2_grasp_controller = builder.AddNamedSystem('iiwa2_grasp_controller', GraspController(plant_iiwa2))

    # Connect ports
    builder.Connect(station.GetOutputPort('iiwa1.position_measured'), iiwa1_traj_controller.GetInputPort('q_actual'))
    builder.Connect(station.GetOutputPort('iiwa2.position_measured'), iiwa2_traj_controller.GetInputPort('q_actual'))
    builder.Connect(iiwa1_traj_controller.GetOutputPort('q_desired'), station.GetInputPort('iiwa1.position'))
    builder.Connect(iiwa2_traj_controller.GetOutputPort('q_desired'), station.GetInputPort('iiwa2.position'))
    builder.Connect(station.GetOutputPort('wsg1.state_measured'), iiwa1_grasp_controller.GetInputPort('q_actual'))
    builder.Connect(station.GetOutputPort('wsg2.state_measured'), iiwa2_grasp_controller.GetInputPort('q_actual'))
    builder.Connect(iiwa1_grasp_controller.GetOutputPort('q_desired'), station.GetInputPort('wsg1.position'))
    builder.Connect(iiwa2_grasp_controller.GetOutputPort('q_desired'), station.GetInputPort('wsg2.position'))

    # Build diagram
    diagram = builder.Build()

    return diagram, station