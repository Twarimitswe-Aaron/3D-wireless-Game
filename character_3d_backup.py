"""
3D Car Driving Game with Arduino Bluetooth Joystick Control
------------------------------------------------
Requirements:
    pip install pygame PyOpenGL pyserial

Controls:
    - Arrow keys or Arduino Bluetooth joystick
    - Up: move forward
    - Down: move backward
    - Left/Right: steer
    - R: reset position
"""

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import math
import serial
import serial.tools.list_ports
import threading
import queue
import time
import re

class Car3D:
    """A 3D car model with physics and controls"""
    def __init__(self):
        self.position = [0, 0, -10]
        self.rotation = [0, 0, 0]  # [pitch, yaw, roll]
        self.velocity = [0, 0, 0]  # Current velocity in x, y, z
        self.speed = 0
        self.max_speed = 1.5  # Increased max speed
        self.turn_speed = 3.0  # More responsive turning
        self.acceleration = 0.05  # Faster acceleration
        self.braking = 0.1  # Braking force
        self.drift_factor = 0.95  # How much the car slides
        self.steering_angle = 0
        self.max_steering = 35  # More steering angle
        self.wheel_rotation = 0
        self.boost = 1.0  # Boost multiplier
        self.boost_amount = 2.0  # Maximum boost multiplier
        self.boost_recharge = 0.01  # Boost recharge rate
        self.boost_consumption = 0.05  # Boost consumption rate
        self.is_boosting = False
        
        # Car colors
        self.colors = {
            'body': (0.2, 0.2, 0.8),     # Blue
            'windows': (0.3, 0.3, 0.4),  # Dark gray
            'wheels': (0.1, 0.1, 0.1),   # Black
            'headlights': (1.0, 1.0, 0.8),# Yellow-white
            'accent': (0.8, 0.8, 0.8)    # Light gray
        }

    def set_boost(self, boosting):
        """Set boost state"""
        self.is_boosting = boosting

    def draw_cube(self, width, height, depth, color):
        """Draw a colored cube"""
        glPushMatrix()
        glScalef(width/2, height/2, depth/2)
        glBegin(GL_QUADS)
        glColor3f(*color)
        # Front
        glVertex3f(-1, -1, 1); glVertex3f(1, -1, 1)
        glVertex3f(1, 1, 1); glVertex3f(-1, 1, 1)
        # Back
        glVertex3f(-1, -1, -1); glVertex3f(-1, 1, -1)
        glVertex3f(1, 1, -1); glVertex3f(1, -1, -1)
        # Top
        glVertex3f(-1, 1, -1); glVertex3f(-1, 1, 1)
        glVertex3f(1, 1, 1); glVertex3f(1, 1, -1)
        # Bottom
        glVertex3f(-1, -1, -1); glVertex3f(1, -1, -1)
        glVertex3f(1, -1, 1); glVertex3f(-1, -1, 1)
        # Left
        glVertex3f(-1, -1, -1); glVertex3f(-1, -1, 1)
        glVertex3f(-1, 1, 1); glVertex3f(-1, 1, -1)
        # Right
        glVertex3f(1, -1, -1); glVertex3f(1, 1, -1)
        glVertex3f(1, 1, 1); glVertex3f(1, -1, 1)
        glEnd()
        glPopMatrix()

    def draw_cylinder(self, radius, height, color):
        """Draw a colored cylinder"""
        glPushMatrix()
        quad = gluNewQuadric()
        glColor3f(*color)
        glRotatef(90, 1, 0, 0)  # Rotate to make cylinder horizontal
        gluCylinder(quad, radius, radius, height, 20, 1)
        # Draw the top and bottom caps
        glPushMatrix()
        gluDisk(quad, 0, radius, 20, 1)
        glTranslatef(0, 0, height)
        gluDisk(quad, 0, radius, 20, 1)
        glPopMatrix()
        glPopMatrix()

    def draw_sphere(self, radius, color):
        """Draw a colored sphere"""
        glPushMatrix()
        quad = gluNewQuadric()
        glColor3f(*color)
        gluSphere(quad, radius, 20, 20)
        glPopMatrix()

    def draw_wheel(self, x, z, steering=0):
        """Draw a wheel at position (x, z) with optional steering"""
        glPushMatrix()
        
        # Position the wheel relative to car
        glTranslatef(x, -0.3, z)
        
        # Apply steering rotation (only for front wheels)
        if abs(steering) > 0.1:  # Only apply steering if significant
            glRotatef(steering, 0, 1, 0)
        
        # Calculate wheel rotation based on car's movement
        car_speed = math.sqrt(self.velocity[0]**2 + self.velocity[2]**2)
        if car_speed > 0.01:  # Only rotate if car is moving
            # Calculate rotation based on distance traveled (circumference = 2*pi*r)
            wheel_radius = 0.4
            circumference = 2 * math.pi * wheel_radius
            distance_this_frame = car_speed * 0.05  # Scale factor for better visualization
            rotation_degrees = (distance_this_frame / circumference) * 360
            
            # Determine rotation direction based on car's movement
            rotation_rad = math.radians(self.rotation[1])
            forward = [math.sin(rotation_rad), 0, math.cos(rotation_rad)]
            dot = forward[0] * self.velocity[0] + forward[2] * self.velocity[2]
            if dot < 0:  # Moving backward
                rotation_degrees = -rotation_degrees
            
            self.wheel_rotation += rotation_degrees
            self.wheel_rotation %= 360  # Keep within 0-360 degrees
        
        # Draw wheel (cylinder along X axis)
        glPushMatrix()
        
        # Rotate cylinder to be horizontal (along X axis)
        glRotatef(90, 0, 1, 0)
        
        # Apply wheel rotation (rotate around X axis for rolling)
        glRotatef(self.wheel_rotation, 1, 0, 0)
        
        # Draw tire (main cylinder)
        self.draw_cylinder(0.4, 0.2, self.colors['wheels'])
        
        # Draw wheel hub (disks on both sides)
        glPushMatrix()
        glColor3f(0.4, 0.4, 0.4)  # Gray color for hub
        
        # Draw inner hub
        glPushMatrix()
        glTranslatef(0, 0, -0.1)  # Move to inner side
        gluDisk(gluNewQuadric(), 0, 0.15, 16, 1)
        glPopMatrix()
        
        # Draw outer hub
        glPushMatrix()
        glTranslatef(0, 0, 0.1)  # Move to outer side
        gluDisk(gluNewQuadric(), 0, 0.15, 16, 1)
        glPopMatrix()
        
        # Draw axle (smaller cylinder through the middle)
        glColor3f(0.2, 0.2, 0.2)
        glRotatef(90, 1, 0, 0)  # Rotate to align with wheel
        self.draw_cylinder(0.08, 0.4, (0.2, 0.2, 0.2))
        
        glPopMatrix()  # End hub drawing
        glPopMatrix()  # End wheel rotation
        glPopMatrix()  # End wheel transform

    def steer(self, direction):
        """Steer the car in the given direction with speed sensitivity"""
        # Calculate current speed
        current_speed = math.sqrt(self.velocity[0]**2 + self.velocity[2]**2)
        
        # More responsive steering at lower speeds
        speed_factor = 0.6 + (1.0 - min(1.0, current_speed / (self.max_speed * 0.7))) * 0.4
        
        # Calculate target steering angle
        target_steer = direction * self.max_steering * speed_factor
        
        # Calculate steering speed based on how far we need to turn
        # Faster steering when we're far from target, slower when close
        steering_diff = target_steer - self.steering_angle
        steering_speed = 0.15 + min(0.25, abs(steering_diff) / self.max_steering * 0.5)
        
        # Apply steering with speed sensitivity
        self.steering_angle += steering_diff * steering_speed
        
        # Limit maximum steering angle
        self.steering_angle = max(-self.max_steering, min(self.max_steering, self.steering_angle))
        
        # Apply self-centering when not steering
        if abs(direction) < 0.1:
            # Faster centering at higher speeds
            centering_speed = 0.1 + min(0.2, current_speed / self.max_speed * 0.3)
            self.steering_angle *= (1.0 - centering_speed)
            
            # Snap to center when close
            if abs(self.steering_angle) < 0.5:
                self.steering_angle = 0
        
        # Add some artificial understeer at high speeds
        if current_speed > self.max_speed * 0.7:
            speed_reduction = (current_speed - self.max_speed * 0.7) / (self.max_speed * 0.3)
            self.steering_angle *= (1.0 - speed_reduction * 0.5)
            
            # Update wheel rotation based on steering and speed
            if current_speed > 0.1:
                # Calculate how much the wheels should turn based on steering and speed
                turn_factor = abs(direction) * 0.5  # How much to turn the wheels
                if direction != 0:
                    # Slight wheel rotation effect when turning
                    turn_speed = 1.5 * (1.0 + min(1.0, current_speed / (self.max_speed * 0.5)))
                    self.wheel_rotation += direction * turn_speed

    def update(self):
        """Update car position and physics with improved handling"""
        # Calculate current speed
        current_speed = math.sqrt(self.velocity[0]**2 + self.velocity[2]**2)
        
        # Apply drag (air resistance and rolling resistance)
        drag = 0.98 - (current_speed / self.max_speed) * 0.02
        self.velocity[0] *= drag
        self.velocity[2] *= drag
        
        # Get movement vectors
        rad = math.radians(self.rotation[1])
        forward = [math.sin(rad), 0, math.cos(rad)]
        right = [math.sin(rad + math.pi/2), 0, math.cos(rad + math.pi/2)]
        
        # Only apply turning forces if we have significant speed
        if current_speed > 0.05 and abs(self.steering_angle) > 0.5:
            # Calculate effective steering angle based on speed (less effective at high speed)
            speed_factor = 1.0 - min(1.0, current_speed / (self.max_speed * 0.8))
            effective_steering = self.steering_angle * speed_factor
            
            # Calculate turning radius based on steering angle and speed
            # More steering = smaller radius, but not zero
            max_turn_angle_rad = math.radians(abs(effective_steering))
            # Adjust turn radius based on speed (tighter turns at lower speeds)
            speed_factor = max(0.5, 1.0 - (current_speed / (self.max_speed * 1.5)))
            turn_radius = (1.0 / (math.sin(max_turn_angle_rad) + 0.0001)) * speed_factor
            turn_radius = max(1.0, min(turn_radius, 30.0))  # Clamp radius
            
            # Calculate angular velocity (in radians)
            angular_velocity = (current_speed / turn_radius) * (1.0 if self.steering_angle > 0 else -1.0)
            
            # Add some understeer effect at higher speeds
            understeer = min(1.0, current_speed / (self.max_speed * 0.7))
            angular_velocity *= (1.0 - understeer * 0.5)
            
            # Update car rotation based on steering and speed
            turn_factor = 0.5 * (1.0 + min(1.0, current_speed / self.max_speed))
            self.rotation[1] += math.degrees(angular_velocity) * turn_factor
            
            # Update movement vectors after rotation
            rad = math.radians(self.rotation[1])
            forward = [math.sin(rad), 0, math.cos(rad)]
            right = [math.sin(rad + math.pi/2), 0, math.cos(rad + math.pi/2)]
            
            # Calculate current movement direction
            if current_speed > 0.01:
                move_dir = [self.velocity[0]/current_speed, 0, self.velocity[2]/current_speed]
                
                # Calculate how much we're moving forward vs sideways
                forward_speed = (self.velocity[0] * forward[0] + 
                                self.velocity[2] * forward[2])
                
                # Dampen lateral movement (drift) based on speed and steering
                speed_factor = min(1.0, current_speed / (self.max_speed * 0.8))
                steering_factor = abs(effective_steering) / self.max_steering
                lateral_damping = 0.97 - (steering_factor * 0.1) - (speed_factor * 0.07)
                
                # Update velocity to maintain forward movement while allowing controlled slides
                self.velocity[0] = forward[0] * forward_speed * 0.98 + right[0] * (self.velocity[0] * right[0] + self.velocity[2] * right[2]) * lateral_damping
                self.velocity[2] = forward[2] * forward_speed * 0.98 + right[2] * (self.velocity[0] * right[0] + self.velocity[2] * right[2]) * lateral_damping
        
        # Update position based on velocity
        self.position[0] += self.velocity[0]
        self.position[2] += self.velocity[2]
        
        # Update speed value for display
        self.speed = math.sqrt(self.velocity[0]**2 + self.velocity[2]**2)
        
        # Update position based on velocity
        self.position[0] += self.velocity[0]
        self.position[2] += self.velocity[2]
        
        # Apply some friction to simulate rolling resistance
        self.velocity[0] *= 0.995
        self.velocity[2] *= 0.995
        
        # Reset steering when no input
        if abs(self.steering_angle) > 0.1:
            self.steering_angle *= 0.85
        else:
            self.steering_angle = 0
            
        # Update boost
        if not self.is_boosting and self.boost < self.boost_amount:
            self.boost = min(self.boost_amount, self.boost + self.boost_recharge)
        
        # Update speed value for display
        self.speed = current_speed

    def brake(self, strength=1.0):
        """Apply brakes to slow down the car"""
        # Calculate current speed
        current_speed = math.sqrt(self.velocity[0]**2 + self.velocity[2]**2)
        
        if current_speed > 0.01:  # Only apply brakes if moving
            # Calculate braking force (stronger when moving faster)
            brake_force = min(self.braking * strength * 2, current_speed * 0.2)
            
            # Calculate direction opposite to current velocity
            if current_speed > 0:
                brake_x = (self.velocity[0] / current_speed) * brake_force
                brake_z = (self.velocity[2] / current_speed) * brake_force
                
                # Apply braking force
                self.velocity[0] -= brake_x
                self.velocity[2] -= brake_z
                
                # Prevent tiny movements
                if abs(self.velocity[0]) < 0.01:
                    self.velocity[0] = 0
                if abs(self.velocity[2]) < 0.01:
                    self.velocity[2] = 0
        else:
            # If already stopped, ensure velocity is exactly zero
            self.velocity[0] = 0
            self.velocity[2] = 0
    
    def accelerate(self, direction):
        """Accelerate the car in the given direction with boost support"""
        rad = math.radians(self.rotation[1])
        acceleration = self.acceleration
        
        # Apply boost if available and boosting
        if direction > 0 and self.is_boosting and self.boost > 0:
            acceleration *= self.boost_amount
            self.boost = max(0, self.boost - self.boost_consumption)
        
        # Calculate acceleration vector
        accel_x = math.sin(rad) * acceleration * direction
        accel_z = math.cos(rad) * acceleration * direction
        
        # Apply acceleration
        self.velocity[0] += accel_x
        self.velocity[2] += accel_z
        
        # Limit speed
        current_speed = math.sqrt(self.velocity[0]**2 + self.velocity[2]**2)
        if current_speed > self.max_speed * (self.boost_amount if self.is_boosting and direction > 0 else 1.0):
            scale = (self.max_speed * (self.boost_amount if self.is_boosting and direction > 0 else 1.0)) / current_speed
            self.velocity[0] *= scale
            self.velocity[2] *= scale

    def reset(self):
        """Reset car to initial position and state"""
        self.position = [0, 0, -10]
        self.rotation = [0, 0, 0]
        self.velocity = [0, 0, 0]
        self.speed = 0
        self.steering_angle = 0
        self.is_boosting = False
        self.boost = 1.0

    def draw(self):
        """Draw the complete car"""
        glPushMatrix()
        
        # Apply position and rotation
        glTranslatef(self.position[0], self.position[1], self.position[2])
        glRotatef(self.rotation[1], 0, 1, 0)  # Yaw (left/right)
        
        # Car body
        glPushMatrix()
        glTranslatef(0, 0.3, 0)
        
        # Main body
        glPushMatrix()
        glScalef(1.5, 0.5, 3)
        self.draw_cube(1, 1, 1, self.colors['body'])
        glPopMatrix()
        
        # Top
        glPushMatrix()
        glTranslatef(0, 0.5, -0.3)
        glScalef(1.2, 0.4, 1.5)
        self.draw_cube(1, 1, 1, self.colors['body'])
        glPopMatrix()
        
        # Windows
        glPushMatrix()
        glTranslatef(0, 0.7, -0.1)
        glScalef(0.8, 0.3, 0.8)
        self.draw_cube(1, 1, 1, self.colors['windows'])
        glPopMatrix()
        
        # Headlights
        glPushMatrix()
        glTranslatef(0.5, 0.3, 1.3)
        self.draw_sphere(0.15, self.colors['headlights'])
        glTranslatef(-1.0, 0, 0)
        self.draw_sphere(0.15, self.colors['headlights'])
        glPopMatrix()
        
        # Grille
        glPushMatrix()
        glTranslatef(0, 0.2, 1.5)
        glScalef(0.8, 0.3, 0.1)
        self.draw_cube(1, 1, 1, self.colors['accent'])
        glPopMatrix()
        
        glPopMatrix()  # End car body
        
        # Draw wheels with steering on front wheels
        # Front wheels (steerable)
        self.draw_wheel(0.8, 1.0, self.steering_angle)  # Front right
        self.draw_wheel(-0.8, 1.0, self.steering_angle)  # Front left
        
        # Rear wheels (fixed direction)
        self.draw_wheel(0.8, -1.0, 0)  # Rear right
        self.draw_wheel(-0.8, -1.0, 0)  # Rear left
        
        glPopMatrix()  # End car transform


class BluetoothReceiver:
    """Handles Bluetooth serial communication in a separate thread"""
    
    def __init__(self):
        self.serial_conn = None
        self.command_queue = queue.Queue()
        self.running = False
        self.connected = False
        self.port = None
        self.last_data = {
            'x': 512,  # Center position for X axis
            'y': 512,  # Center position for Y axis
            'button': 0  # Button state
        }
        
    def find_port(self):
        """Find available serial ports"""
        ports = serial.tools.list_ports.comports()
        available_ports = []
        
        print("\n" + "="*60)
        print("Available Serial Ports:")
        print("="*60)
        
        for i, port in enumerate(ports):
            print(f"  [{i}] {port.device} - {port.description}")
            available_ports.append(port.device)
            
            # Auto-detect common Bluetooth devices
            if 'HC-05' in port.description or 'HC-06' in port.description or 'Bluetooth' in port.description:
                print(f"      ^ Bluetooth device detected!")
                self.port = port.device
        
        if not available_ports:
            print("  No serial ports found!")
            print("\nMake sure:")
            print("  1. Arduino/Bluetooth is connected")
            print("  2. Drivers are installed")
            print("  3. Device is paired (for Bluetooth)")
            return None
            
        return available_ports
    
    def connect(self, port=None, baudrate=9600):
        """Connect to serial port"""
        if port:
            self.port = port
        
        if not self.port:
            available = self.find_port()
            if not available:
                return False
                
            if not self.port:  # Still not auto-detected
                try:
                    print("\nEnter port number to use (or 'q' to skip): ", end='')
                    choice = input().strip()
                    if choice.lower() == 'q':
                        print("Skipping Bluetooth connection - keyboard control only")
                        return False
                    idx = int(choice)
                    if 0 <= idx < len(available):
                        self.port = available[idx]
                except (ValueError, IndexError):
                    print("Invalid selection")
                    return False
        
        try:
            print(f"\nConnecting to {self.port} at {baudrate} baud...")
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=baudrate,
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            time.sleep(2)  # Wait for connection to stabilize
            self.connected = True
            print("✓ Bluetooth connection established!")
            print("  Listening for joystick commands...\n")
            return True
        except serial.SerialException as e:
            print(f"✗ Connection failed: {e}")
            print("\nTroubleshooting:")
            print("  1. Check if device is paired")
            print("  2. Close other programs using the port")
            print("  3. Try different baud rates (9600, 38400, 115200)")
            print("  4. Verify Arduino is powered on")
            return False
    
    def start_listening(self):
        """Start listening thread"""
        if not self.connected:
            return False
            
        self.running = True
        listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
        listen_thread.start()
        return True
    
    def _parse_arduino_data(self, line):
        """Parse Arduino joystick data in various formats"""
        # Try to parse different data formats from Arduino
        # Format 1: "X:512 Y:512 B:0"
        # Format 2: "512,512,0"
        # Format 3: "X512Y512B0"
        
        line = line.strip()
        
        # Remove any non-printable characters
        line = ''.join(char for char in line if char.isprintable())
        
        # Try format 1: "X:512 Y:512 B:0"
        match = re.search(r'X:?(\d+)\s*Y:?(\d+)\s*B:?(\d+)', line, re.IGNORECASE)
        if match:
            return {
                'x': int(match.group(1)),
                'y': int(match.group(2)),
                'button': int(match.group(3))
            }
        
        # Try format 2: "512,512,0"
        parts = line.split(',')
        if len(parts) >= 3:
            try:
                return {
                    'x': int(parts[0]),
                    'y': int(parts[1]),
                    'button': int(parts[2])
                }
            except ValueError:
                pass
        
        # Try format 3: Simple numbers separated by spaces
        parts = line.split()
        if len(parts) >= 3:
            try:
                return {
                    'x': int(parts[0]),
                    'y': int(parts[1]),
                    'button': int(parts[2])
                }
            except ValueError:
                pass
        
        return None
    
    def _listen_loop(self):
        """Main listening loop (runs in separate thread)"""
        print("Bluetooth listener thread started")
        buffer = ""
        
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    # Read all available data
                    data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line:
                            # Parse Arduino data
                            parsed_data = self._parse_arduino_data(line)
                            if parsed_data:
                                self.last_data = parsed_data
                                #print(f"BT → X:{parsed_data['x']} Y:{parsed_data['y']} B:{parsed_data['button']}")
                                self.command_queue.put(parsed_data)
                            else:
                                print(f"BT → Unparsed: {line}")
            
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                self.connected = False
                break
            except Exception as e:
                print(f"Error in Bluetooth thread: {e}")
            
            time.sleep(0.01)
    
    def get_command(self):
        """Get next command from queue (non-blocking)"""
        try:
            return self.command_queue.get_nowait()
        except queue.Empty:
            return None
    
    def get_joystick_state(self):
        """Get current joystick state with deadzone applied"""
        return self.last_data.copy()
    
    def stop(self):
        """Stop listening and close connection"""
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Bluetooth connection closed")


class Car3DViewer:
    """Main viewer class for the 3D car with Bluetooth control"""
    def __init__(self):
        pygame.init()
        self.display = (1366, 768)  # Wider display for better view
        pygame.display.set_mode(self.display, DOUBLEBUF | OPENGL)
        pygame.display.set_caption("Asphalt-Style 3D Racing")
        
        # Joystick history for smoothing
        self.joystick_history = []
        for _ in range(3):
            self.joystick_history.append((512, 512, 0))  # Center position
        
        # Setup OpenGL
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        
        glClearColor(0.5, 0.7, 1.0, 1.0)  # Sky blue background
        
        # Setup lighting
        glLightfv(GL_LIGHT0, GL_POSITION, (5, 10, 5, 1))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.5, 0.5, 0.5, 1))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (1.0, 1.0, 1.0, 1))
        glLightfv(GL_LIGHT0, GL_SPECULAR, (1.0, 1.0, 1.0, 1))
        
        # Set up the projection matrix
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(60, (self.display[0] / self.display[1]), 0.1, 100.0)
        
        # Switch to modelview matrix
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        
        # Create car instance
        self.car = Car3D()
        self.clock = pygame.time.Clock()
        self.running = True
        
        # Initialize Bluetooth receiver
        self.bt_receiver = BluetoothReceiver()
        
        # Font for status display
        pygame.font.init()
        self.font = pygame.font.Font(None, 24)
        
        # Camera settings for different views
        self.camera_modes = {
            'chase': {
                'distance': 8.0,
                'height': 2.5,
                'angle': 180.0,
                'lerp_speed': 0.1,
                'look_ahead': 5.0
            },
            'hood': {
                'distance': 1.5,
                'height': 0.8,
                'angle': 0.0,
                'lerp_speed': 0.2,
                'look_ahead': 10.0
            },
            'far': {
                'distance': 15.0,
                'height': 8.0,
                'angle': 200.0,
                'lerp_speed': 0.08,
                'look_ahead': 15.0
            },
            'orbit': {
                'distance': 12.0,
                'height': 4.0,
                'angle': 0.0,
                'lerp_speed': 0.05,
                'look_ahead': 8.0
            }
        }
        
        # Initialize camera state
        self.camera_mode = 'chase'  # Start with chase camera
        self.camera_distance = self.camera_modes[self.camera_mode]['distance']
        self.camera_height = self.camera_modes[self.camera_mode]['height']
        self.camera_angle = self.camera_modes[self.camera_mode]['angle']
        self.camera_lerp_speed = self.camera_modes[self.camera_mode]['lerp_speed']
        self.camera_position = [0, 5, -10]
        self.camera_target = [0, 0, 0]
        self.camera_look_ahead = self.camera_modes[self.camera_mode]['look_ahead']
        self.orbit_angle = 0.0
        
        # Joystick settings
        self.deadzone = 100  # Deadzone for joystick center
        self.joystick_center = 512  # Center value for joystick
        
        # Movement state
        self.move_forward = False
        self.move_backward = False
        self.turn_left = False
        self.turn_right = False

    def handle_arduino_joystick(self, joystick_data):
        """Handle Arduino joystick data with proper deadzone and smooth controls"""
        try:
            x = int(joystick_data.get('x', self.joystick_center))
            y = int(joystick_data.get('y', self.joystick_center))
            button = int(joystick_data.get('button', 0))
            
            # Smooth out the input using a simple moving average
            self.joystick_history.append((x, y, button))
            if len(self.joystick_history) > 3:  # Keep last 3 readings
                self.joystick_history.pop(0)
                
            # Average the readings
            x = sum(h[0] for h in self.joystick_history) // len(self.joystick_history)
            y = sum(h[1] for h in self.joystick_history) // len(self.joystick_history)
            button = 1 if any(h[2] for h in self.joystick_history) else 0
            
            # Apply deadzone to X axis (steering)
            if abs(x - self.joystick_center) < self.deadzone:
                steer_input = 0
            else:
                # Apply a more responsive curve to steering
                steer_input = (x - self.joystick_center) / (1023 - self.joystick_center)
                steer_input = math.copysign(steer_input * abs(steer_input), steer_input)  # Quadratic response
                steer_input = max(-1, min(1, steer_input))
            
            # Apply deadzone to Y axis (acceleration/brake)
            if abs(y - self.joystick_center) < self.deadzone:
                accel_input = 0
            else:
                # More sensitive acceleration in both directions
                accel_input = (self.joystick_center - y) / (1023 - self.joystick_center)
                accel_input = math.copysign(accel_input * abs(accel_input), accel_input)  # Quadratic response
                accel_input = max(-1, min(1, accel_input))
            
            # Apply steering with speed sensitivity
            speed_factor = 0.5 + (1.0 - min(1.0, self.car.speed / (self.car.max_speed * 0.7))) * 0.5
            self.car.steer(steer_input * speed_factor)
            
            # Handle acceleration/braking with better response
            current_speed = math.sqrt(self.car.velocity[0]**2 + self.car.velocity[2]**2)
            
            # If we're trying to go in the opposite direction, brake first
            if (accel_input > 0.1 and current_speed < -0.5) or (accel_input < -0.1 and current_speed > 0.5):
                self.car.brake(0.5)
            else:
                # Apply acceleration in the desired direction
                if abs(accel_input) > 0.1:
                    self.car.accelerate(accel_input * (2.0 if button == 1 else 1.0))  # Boost with button
                else:
                    # Gentle braking when no input
                    if abs(current_speed) > 0.1:
                        self.car.brake(0.1)
            
            # Update movement states for camera and UI
            self.move_forward = accel_input > 0.1
            self.move_backward = accel_input < -0.1
            
            # Handle boost with the button
            self.car.set_boost(button == 1)
            
            return steer_input, accel_input, button
            
        except Exception as e:
            print(f"Error in joystick handling: {e}")
            return 0, 0, 0

    def draw_road(self):
        """Draw the road and environment"""
        glDisable(GL_LIGHTING)
        
        # Draw road surface
        glBegin(GL_QUADS)
        # Road
        glColor3f(0.2, 0.2, 0.2)  # Dark gray
        glVertex3f(-5, 0, -100)
        glVertex3f(5, 0, -100)
        glVertex3f(5, 0, 100)
        glVertex3f(-5, 0, 100)
        
        # Road shoulders
        glColor3f(0.3, 0.6, 0.3)  # Green grass
        # Left shoulder
        glVertex3f(-15, -0.1, -100)
        glVertex3f(-5, -0.1, -100)
        glVertex3f(-5, -0.1, 100)
        glVertex3f(-15, -0.1, 100)
        # Right shoulder
        glVertex3f(5, -0.1, -100)
        glVertex3f(15, -0.1, -100)
        glVertex3f(15, -0.1, 100)
        glVertex3f(5, -0.1, 100)
        glEnd()
        
        # Draw road markings
        glBegin(GL_QUADS)
        glColor3f(1.0, 1.0, 1.0)  # White
        for z in range(-100, 100, 5):
            glVertex3f(-0.2, 0.01, z)
            glVertex3f(0.2, 0.01, z)
            glVertex3f(0.2, 0.01, z + 2)
            glVertex3f(-0.2, 0.01, z + 2)
        glEnd()
        
        glEnable(GL_LIGHTING)
    
    def draw_boost_meter(self):
        """Draw the boost meter on screen using immediate mode rendering"""
        try:
            # Skip if we don't have the required attributes
            if not hasattr(self.car, 'boost') or not hasattr(self.car, 'boost_amount') or self.car.boost_amount <= 0:
                return
                
            # Save current state
            lighting = glIsEnabled(GL_LIGHTING)
            depth_test = glIsEnabled(GL_DEPTH_TEST)
            texture_2d = glIsEnabled(GL_TEXTURE_2D)
            
            # Set up for 2D rendering
            glDisable(GL_LIGHTING)
            glDisable(GL_DEPTH_TEST)
            glDisable(GL_TEXTURE_2D)
            
            # Save current matrices
            glMatrixMode(GL_PROJECTION)
            glPushMatrix()
            glLoadIdentity()
            glOrtho(0, self.display[0], self.display[1], 0, -1, 1)
            
            glMatrixMode(GL_MODELVIEW)
            glPushMatrix()
            glLoadIdentity()
            
            # Draw boost meter background
            meter_width = 200
            meter_height = 20
            meter_x = 20
            meter_y = self.display[1] - 40
            
            # Background
            glColor4f(0.2, 0.2, 0.2, 0.7)
            glBegin(GL_QUADS)
            glVertex2f(meter_x, meter_y)
            glVertex2f(meter_x + meter_width, meter_y)
            glVertex2f(meter_x + meter_width, meter_y + meter_height)
            glVertex2f(meter_x, meter_y + meter_height)
            glEnd()
            
            # Boost level
            if self.car.boost_amount > 0:  # Avoid division by zero
                boost_ratio = max(0, min(1, self.car.boost / self.car.boost_amount))
                boost_width = int(meter_width * boost_ratio)
                
                # Draw boost level
                glColor4f(0.0, 0.7, 1.0, 0.9)
                glBegin(GL_QUADS)
                glVertex2f(meter_x, meter_y)
                glVertex2f(meter_x + boost_width, meter_y)
                glVertex2f(meter_x + boost_width, meter_y + meter_height)
                glVertex2f(meter_x, meter_y + meter_height)
                glEnd()
            
            # Border
            glLineWidth(2)
            glColor4f(1.0, 1.0, 1.0, 0.8)
            glBegin(GL_LINE_LOOP)
            glVertex2f(meter_x, meter_y)
            glVertex2f(meter_x + meter_width, meter_y)
            glVertex2f(meter_x + meter_width, meter_y + meter_height)
            glVertex2f(meter_x, meter_y + meter_height)
            glEnd()
            
        except Exception as e:
            print(f"Error in draw_boost_meter: {e}")
            
        finally:
            # Always restore matrices and state, even if an error occurs
            try:
                glMatrixMode(GL_MODELVIEW)
                glPopMatrix()  # Pop modelview
                glMatrixMode(GL_PROJECTION)
                glPopMatrix()  # Pop projection
                glMatrixMode(GL_MODELVIEW)  # Switch back to modelview mode
                
                # Restore OpenGL state
                if lighting:
                    glEnable(GL_LIGHTING)
                if depth_test:
                    glEnable(GL_DEPTH_TEST)
                if texture_2d:
                    glEnable(GL_TEXTURE_2D)
            except Exception as e:
                print(f"Error restoring OpenGL state: {e}")
                # Reset to a known state
                glMatrixMode(GL_MODELVIEW)
                glLoadIdentity()
    
    def draw_status_overlay(self):
        """Draw status information on screen"""
        try:
            # Save current state
            lighting = glIsEnabled(GL_LIGHTING)
            depth_test = glIsEnabled(GL_DEPTH_TEST)
            texture_2d = glIsEnabled(GL_TEXTURE_2D)
            
            # Set up for 2D rendering
            glDisable(GL_LIGHTING)
            glDisable(GL_DEPTH_TEST)
            glDisable(GL_TEXTURE_2D)
            
            # Save current matrices
            glMatrixMode(GL_PROJECTION)
            glPushMatrix()
            glLoadIdentity()
            gluOrtho2D(0, self.display[0], self.display[1], 0)
            
            glMatrixMode(GL_MODELVIEW)
            glPushMatrix()
            glLoadIdentity()
            
            # Draw semi-transparent background
            glColor4f(0.0, 0.0, 0.0, 0.5)
            glBegin(GL_QUADS)
            glVertex2f(10, 10)
            glVertex2f(400, 10)
            glVertex2f(400, 180)
            glVertex2f(10, 180)
            glEnd()
            
            # Get joystick state for display
            joystick_data = self.bt_receiver.get_joystick_state()
            bt_status = "Connected" if self.bt_receiver.connected else "Disconnected"
            
            # Calculate boost percentage safely
            try:
                boost_pct = int((self.car.boost / self.car.boost_amount) * 100)
            except (AttributeError, ZeroDivisionError):
                boost_pct = 0
            
            # Draw status text
            status = [
                f"Speed: {abs(self.car.speed * 100):.1f} km/h",
                f"Boost: {boost_pct}%",
                f"Position: ({getattr(self.car, 'position', [0, 0, 0])[0]:.1f}, {getattr(self.car, 'position', [0, 0, 0])[2]:.1f})",
                f"Camera: {getattr(self, 'camera_mode', 'chase').capitalize()}",
                f"Bluetooth: {bt_status}",
                f"Joystick: X:{joystick_data.get('x', 0)} Y:{joystick_data.get('y', 0)} B:{joystick_data.get('button', 0)}",
                "",
                "Controls:",
                "[C] Change Camera  [R] Reset Position",
                "[SPACE] Handbrake  [SHIFT] Boost",
                "Joystick: Forward/Back + Left/Right"
            ]
            
            for i, line in enumerate(status):
                try:
                    text_surface = self.font.render(line, True, (255, 255, 255))
                    text_data = pygame.image.tostring(text_surface, "RGBA", True)
                    glRasterPos2i(20, 30 + i * 20)
                    glDrawPixels(text_surface.get_width(), text_surface.get_height(),
                                GL_RGBA, GL_UNSIGNED_BYTE, text_data)
                except Exception as e:
                    print(f"Error rendering text: {e}")
            
        except Exception as e:
            print(f"Error in draw_status_overlay: {e}")
            
        finally:
            # Always restore matrices and state, even if an error occurs
            try:
                # Restore modelview matrix
                glMatrixMode(GL_MODELVIEW)
                glPopMatrix()
                
                # Restore projection matrix
                glMatrixMode(GL_PROJECTION)
                glPopMatrix()
                
                # Switch back to modelview mode
                glMatrixMode(GL_MODELVIEW)
                
                # Restore OpenGL state
                if lighting:
                    glEnable(GL_LIGHTING)
                if depth_test:
                    glEnable(GL_DEPTH_TEST)
                if texture_2d:
                    glEnable(GL_TEXTURE_2D)
                    
            except Exception as e:
                print(f"Error restoring OpenGL state in draw_status_overlay: {e}")
                # Reset to a known state
                glMatrixMode(GL_MODELVIEW)
                glLoadIdentity()
    
    def update_camera(self):
        """Update camera position based on car movement and camera mode"""
        mode = self.camera_modes[self.camera_mode]
        car_rad = math.radians(self.car.rotation[1])
        
        if self.camera_mode == 'orbit':
            # Update orbit angle
            self.orbit_angle = (self.orbit_angle + 0.2) % 360
            rad = math.radians(self.orbit_angle)
            
            # Calculate circular orbit position
            target_x = self.car.position[0] + math.sin(rad) * mode['distance']
            target_z = self.car.position[2] + math.cos(rad) * mode['distance']
            target_y = self.car.position[1] + mode['height']
            
            # Look at car
            look_x = self.car.position[0]
            look_y = self.car.position[1] + 0.5
            look_z = self.car.position[2]
            
        else:
            # Calculate direction vector based on car's movement for look-ahead
            speed = math.sqrt(self.car.velocity[0]**2 + self.car.velocity[2]**2)
            look_ahead_factor = min(1.0, speed / 5.0)  # How much to look ahead based on speed
            
            # Calculate camera position
            rad = math.radians(self.car.rotation[1] + mode['angle'])
            target_x = self.car.position[0] - math.sin(rad) * mode['distance']
            target_z = self.car.position[2] - math.cos(rad) * mode['distance']
            target_y = self.car.position[1] + mode['height']
            
            # Calculate look-ahead point
            look_ahead = mode['look_ahead'] * look_ahead_factor
            look_x = self.car.position[0] + math.sin(math.radians(self.car.rotation[1])) * look_ahead
            look_y = self.car.position[1] + 0.5
            look_z = self.car.position[2] + math.cos(math.radians(self.car.rotation[1])) * look_ahead
        
        # Smoothly interpolate camera position
        self.camera_position[0] += (target_x - self.camera_position[0]) * mode['lerp_speed']
        self.camera_position[1] += (target_y - self.camera_position[1]) * mode['lerp_speed']
        self.camera_position[2] += (target_z - self.camera_position[2]) * mode['lerp_speed']
        
        # Update camera target with smoothing
        self.camera_target[0] += (look_x - self.camera_target[0]) * mode['lerp_speed'] * 2
        self.camera_target[1] += (look_y - self.camera_target[1]) * mode['lerp_speed'] * 2
        self.camera_target[2] += (look_z - self.camera_target[2]) * mode['lerp_speed'] * 2
        
        # Set up the view matrix
        glLoadIdentity()
        
        # Handle different camera modes
        if self.camera_mode == 'orbit':
            # Orbit camera - rotates around the car
            self.orbit_angle = (self.orbit_angle + 0.5) % 360
            rad = math.radians(self.orbit_angle)
            
            cam_x = self.car.position[0] + math.sin(rad) * self.camera_distance
            cam_z = self.car.position[2] + math.cos(rad) * self.camera_distance
            cam_y = self.car.position[1] + self.camera_height
            
            gluLookAt(cam_x, cam_y, cam_z,
                     self.car.position[0], 
                     self.car.position[1] + 1.0, 
                     self.car.position[2],
                     0, 1, 0)
        else:
            # For other camera modes (chase, hood, far)
            if abs(self.car.steering_angle) > 5.0:
                # Calculate tilt based on steering and speed
                tilt = math.sin(math.radians(self.car.steering_angle * 2)) * 0.2
                up_x = math.sin(tilt)
                up_y = math.cos(tilt)
                gluLookAt(
                    self.camera_position[0], self.camera_position[1], self.camera_position[2],
                    self.camera_target[0], self.camera_target[1], self.camera_target[2],
                    up_x, up_y, 0  # Tilted up vector for banking effect
                )
            else:
                gluLookAt(
                    self.camera_position[0], self.camera_position[1], self.camera_position[2],
                    self.camera_target[0], self.camera_target[1], self.camera_target[2],
                    0, 1, 0  # Standard up vector
                )
    
    def run(self):
        """Main rendering loop"""
        try:
            # Try to connect to Bluetooth
            if not self.bt_receiver.connect():
                print("⚠️ Could not connect to Bluetooth device. Using keyboard controls only.")
            else:
                self.bt_receiver.start_listening()
                print("✅ Bluetooth connected. You can now use the Arduino joystick!")
            
            print("\n=== Controls ===")
            print("Joystick: Forward/Back + Left/Right")
            print("Button: Boost")
            print("Keyboard alternatives:")
            print("↑ - Accelerate")
            print("↓ - Reverse")
            print("← → - Steer")
            print("SPACE - Handbrake/Drift")
            print("SHIFT - Boost")
            print("C - Cycle camera views")
            print("R - Reset position")
            print("ESC - Quit\n")
            
            while self.running:
                # Handle events
                for event in pygame.event.get():
                    if event.type == QUIT:
                        self.running = False
                    elif event.type == KEYDOWN:
                        if event.key == K_ESCAPE:
                            self.running = False
                        elif event.key == K_UP:
                            self.move_forward = True
                        elif event.key == K_DOWN:
                            self.move_backward = True
                        elif event.key == K_LEFT:
                            self.turn_left = True
                        elif event.key == K_RIGHT:
                            self.turn_right = True
                        elif event.key == K_LSHIFT or event.key == K_RSHIFT:
                            self.car.set_boost(True)
                        elif event.key == K_s:
                            # Brake to stop
                            self.car.brake(1.0)
                        elif event.key == K_SPACE:
                            # Handbrake/drift
                            self.car.drift_factor = 0.8
                        elif event.key == K_c:
                            # Cycle camera modes
                            modes = ['chase', 'hood', 'orbit']
                            current_idx = modes.index(self.camera_mode) if self.camera_mode in modes else 0
                            self.camera_mode = modes[(current_idx + 1) % len(modes)]
                        elif event.key == K_r:
                            self.car.reset()
                    elif event.type == KEYUP:
                        if event.key == K_UP:
                            self.move_forward = False
                        elif event.key == K_DOWN:
                            self.move_backward = False
                        elif event.key == K_LEFT:
                            self.turn_left = False
                        elif event.key == K_RIGHT:
                            self.turn_right = False
                        elif event.key == K_LSHIFT or event.key == K_RSHIFT:
                            self.car.set_boost(False)
                        elif event.key == K_SPACE:
                            # Release handbrake
                            self.car.drift_factor = 0.95
                
                # Handle continuous key presses
                if self.move_forward:
                    self.car.accelerate(1)
                elif self.move_backward:
                    self.car.accelerate(-1)
                
                if self.turn_left:
                    self.car.steer(-1)
                elif self.turn_right:
                    self.car.steer(1)
                
                # Handle Bluetooth joystick commands
                if self.bt_receiver.connected:
                    # Get current joystick state
                    joystick_data = self.bt_receiver.get_joystick_state()
                    self.handle_arduino_joystick(joystick_data)
                
                # Update car physics
                self.car.update()
                
                # Clear the screen and depth buffer
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
                
                # Reset the modelview matrix
                glLoadIdentity()
                
                # Update camera based on current mode
                self.update_camera()
                
                # Draw the scene
                self.draw_road()
                
                # Draw the car
                glPushMatrix()
                self.car.draw()
                glPopMatrix()
                
                # Draw status overlay with more info
                self.draw_status_overlay()
                
                # Draw boost meter
                self.draw_boost_meter()
                
                # Update the display
                pygame.display.flip()
                
                # Cap the frame rate
                self.clock.tick(60)
                
        finally:
            # Clean up
            self.bt_receiver.stop()
            pygame.quit()


if __name__ == '__main__':
    try:
        viewer = Car3DViewer()
        viewer.run()
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        input("\nPress Enter to exit...")