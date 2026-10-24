#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ctypes
import os
from ament_index_python.packages import get_package_prefix

class D1ArmDriverNode(Node):
    def __init__(self):
        super().__init__('d1_arm_driver')
        self.subscription = self.create_subscription(
            String,
            'd1_arm/command',
            self.command_callback,
            10)
        self.subscription  # prevent unused variable warning

        try:
            self.cpsl_d1_arm_prefix = get_package_prefix('CPSL_D1_Arm')
            self.lib_dir = os.path.join(self.cpsl_d1_arm_prefix, 'lib')
            self.get_logger().info(f"Loaded CPSL_D1_Arm prefix: {self.lib_dir}")
        except Exception as e:
            self.get_logger().error(f"Could not find CPSL_D1_Arm package: {e}")
            self.lib_dir = ""

        self._preload_dds_runtime()

    def _preload_dds_runtime(self):
        """
        Some arm motion libraries depend on DDS runtime symbols that are not
        resolved correctly unless the runtime libraries are loaded in the right
        order. Preload them explicitly before loading any arm shared object.
        """
        dds_libs = [
            '/usr/local/lib/libddsc.so.0',
            '/usr/local/lib/libddscxx.so.0'
        ]

        for lib_path in dds_libs:
            if not os.path.exists(lib_path):
                self.get_logger().warn(f"DDS runtime library not found: {lib_path}")
                continue
            try:
                ctypes.CDLL(lib_path, mode=ctypes.RTLD_GLOBAL)
                self.get_logger().debug(f"Preloaded DDS runtime library: {lib_path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to preload DDS runtime {lib_path}: {e}")

    def command_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f"Received command: {command}")
        
        if not self.lib_dir:
            self.get_logger().error("Library directory not found, cannot execute command.")
            return

        lib_name = f"lib{command}.so"
        lib_path = os.path.join(self.lib_dir, lib_name)
        
        if not os.path.exists(lib_path):
            self.get_logger().error(f"Library not found: {lib_path}")
            return
        
        try:
            self.get_logger().info(f"Loading library {lib_path}...")
            lib = ctypes.CDLL(lib_path)

            if command.startswith('grasp'):
                # grasp functions take an int argument
                func = getattr(lib, 'Grasp')
                func.argtypes = [ctypes.c_int]
                func.restype = None
                self.get_logger().info(f"Executing Grasp(0) from {lib_name}")
                func(0)
            elif command == 'zero':
                func = getattr(lib, 'Zero')
                func.argtypes = []
                func.restype = None
                self.get_logger().info(f"Executing Zero() from {lib_name}")
                func()
            elif command.startswith('left'):
                # left functions e.g. Left_1
                func_name = command.capitalize()
                func = getattr(lib, func_name)
                func.argtypes = []
                func.restype = None
                self.get_logger().info(f"Executing {func_name}() from {lib_name}")
                func()
            elif command.startswith('right'):
                # right functions e.g. Right_1
                func_name = command.capitalize()
                func = getattr(lib, func_name)
                func.argtypes = []
                func.restype = None
                self.get_logger().info(f"Executing {func_name}() from {lib_name}")
                func()
            else:
                self.get_logger().error(f"Unknown command format: {command}")
                
            self.get_logger().info(f"Command {command} executed successfully.")
            
        except Exception as e:
            self.get_logger().error(f"Error executing command {command}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = D1ArmDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
