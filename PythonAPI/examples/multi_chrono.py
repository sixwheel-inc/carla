import carla

# Connect to the client and get the world
client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()
blueprints = world.get_blueprint_library()
spawn_points = world.get_map().get_spawn_points()

# Spawn blue prints for revoy, trailer and tractor.
vehicle_bp = blueprints.filter('vehicle.*model3*')[0]

# Spawn multiple vehicles
vehicles = []
for i in range(3):
    transform = spawn_points[i]
    vehicle = world.spawn_actor(vehicle_bp, transform)
    vehicles.append(vehicle)

# enable multi physics chrono
world.enable_chrono_physics_multi(
    vehicles
)

# TODO: Most of the code copy pastes the KeyboardControl, so we'll just use those.
try:
    while True:
        # Someone needs to drive the tractor here. so we can copy paste the code from other examples.
        world.tick()
finally:
    for vehicle in vehicles:
        vehicle.destroy()