# First import the library
import pyrealsense2 as rs

# Declare pointcloud object, for calculating pointclouds and texture mappings
pc1 = rs.pointcloud()
pc2 = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points1 = rs.points()
points2 = rs.points()

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe1 = rs.pipeline()
pipe2 = rs.pipeline()

config1 = rs.config()
config1.enable_device('141722073646')

# Enable depth and color streams
config1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

config2 = rs.config()
config2.enable_device('140122071889')

config2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming with chosen configuration
pipe1.start(config1)
pipe2.start(config2)

# We'll use the colorizer to generate texture for our PLY
# (alternatively, texture can be obtained from color or infrared stream)
colorizer = rs.colorizer()

try:
    # Wait for the next set of frames from the camera
    frames1 = pipe1.wait_for_frames()
    colorized1 = colorizer.process(frames1)

    frames2 = pipe2.wait_for_frames()
    colorized2 = colorizer.process(frames2)

    # Create save_to_ply object
    ply1 = rs.save_to_ply("toolbin_5_1.ply")
    ply2 = rs.save_to_ply("toolbin_5_2.ply")

    # Set options to the desired values
    # In this example we'll generate a textual PLY with normals (mesh is already created by default)
    ply1.set_option(rs.save_to_ply.option_ply_binary, False)
    ply1.set_option(rs.save_to_ply.option_ply_normals, True)

    ply2.set_option(rs.save_to_ply.option_ply_binary, False)
    ply2.set_option(rs.save_to_ply.option_ply_normals, True)

    print("Saving to 1.ply...")
    # Apply the processing block to the frameset which contains the depth frame and the texture
    ply1.process(colorized1)
    print("Saving to 2.ply...")
    ply2.process(colorized2)
    print("Done")
finally:
    pipe1.stop()
    pipe2.stop()
