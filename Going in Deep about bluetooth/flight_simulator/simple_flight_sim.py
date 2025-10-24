"""
Simple 3D Flight Simulator with Bluetooth Control
Focuses on reading Bluetooth data and displaying airplane movement
FIXED: Resolved OpenGL stack overflow error
"""

import sys
import os
import math
import time
import threading
from collections import deque

# Add current directory to path
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

try:
    import pygame
    from pygame.locals import *
    from OpenGL.GL import *
    from OpenGL.GLU import *
    import serial
    import serial.tools.list_ports
    import re
except ImportError as e:
    print(f"Missing library: {e}")
    print("Install with: pip install pygame PyOpenGL PyOpenGL-accelerate pyserial")
    sys.exit(1)

class SimpleFlightSim:
    """Simple 3D Flight Simulator with Bluetooth Data Reading"""
    
    def __init__(self):
        # Initialize Pygame and OpenGL
        pygame.init()
        self.display = (1200, 800)
        
        # Set OpenGL attributes
        pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLEBUFFERS, 1)
        pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLESAMPLES, 4)
        
        # Create window
        self.screen = pygame.display.set_mode(self.display, DOUBLEBUF | OPENGL)
        pygame.display.set_caption("3D Flight Simulator - Bluetooth Control")
        
        # Initialize OpenGL
        self.init_opengl()
        
        # FIX: Create quadric objects ONCE during initialization
        self.quadric = gluNewQuadric()
        
        # Flight data
        self.position = [0.0, 20.0, -50.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll = 0.0
        self.throttle = 0.5
        
        # Control inputs
        self.target_roll = 0.0
        self.target_pitch = 0.0
        
        # Flight parameters
        self.acceleration = 0.02
        self.lift_coefficient = 0.08
        self.drag = 0.98
        self.gravity = 0.015
        
        # Animation
        self.propeller_rotation = 0.0
        self.propeller_speed = 30.0
        
        # Game state
        self.clock = pygame.time.Clock()
        self.running = True
        self.fps = 60
        
        # Camera
        self.camera_distance = 25.0
        
        # Bluetooth controller
        self.bt_controller = None
        self.bt_connected = False
        self.bt_data = {'x': 512, 'y': 512}
        self.setup_bluetooth()
        
        # Performance tracking
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.current_fps = 0
        
        print("✅ Simple 3D Flight Simulator Initialized")
        print("🎮 Controls: Bluetooth Joystick + Arrow Keys (pitch/roll), W/S (throttle)")
    
    def init_opengl(self):
        """Initialize OpenGL settings"""
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_MULTISAMPLE)
        
        # Set background color (sky blue)
        glClearColor(0.53, 0.81, 0.98, 1.0)
        
        # Lighting
        glLightfv(GL_LIGHT0, GL_POSITION, (50, 100, 50, 1))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.7, 0.7, 0.7, 1))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (1.0, 1.0, 1.0, 1))
        glLightfv(GL_LIGHT0, GL_SPECULAR, (0.3, 0.3, 0.3, 1))
        
        # Projection
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(60, self.display[0] / self.display[1], 0.1, 500.0)
        glMatrixMode(GL_MODELVIEW)
    
    def setup_bluetooth(self):
        """Setup Bluetooth controller"""
        try:
            from bluetooth_controller import BluetoothController
            self.bt_controller = BluetoothController()
            if self.bt_controller.connect():
                self.bt_controller.start_listening()
                self.bt_connected = True
                print("✅ Bluetooth Controller Connected")
            else:
                print("⚠️  Bluetooth Not Found - Using Keyboard")
        except Exception as e:
            print(f"❌ Bluetooth Error: {e}")
    
    def process_bluetooth_input(self):
        """Process Bluetooth joystick input"""
        if self.bt_connected and self.bt_controller:
            try:
                # Get raw data
                raw_data = self.bt_controller.raw_data
                self.bt_data = raw_data
                
                # Calculate position difference from center (512, 512)
                center_x, center_y = 512, 512
                x_diff = raw_data['x'] - center_x
                y_diff = raw_data['y'] - center_y
                
                # Apply deadzone
                deadzone = 50
                if abs(x_diff) < deadzone:
                    x_diff = 0
                if abs(y_diff) < deadzone:
                    y_diff = 0
                
                # Convert to control inputs (-1 to 1)
                max_deviation = 200
                roll_input = x_diff / max_deviation
                pitch_input = -y_diff / max_deviation  # Invert Y axis
                
                # Apply to airplane controls
                self.target_roll = roll_input * 45.0  # Max 45 degrees bank
                self.target_pitch = pitch_input * 30.0  # Max 30 degrees pitch
                
                # Calculate acceleration based on distance from center
                distance = math.sqrt(x_diff**2 + y_diff**2)
                acceleration_factor = 1.0 + (distance / 200.0) * 2.0  # Up to 3x acceleration
                self.throttle = min(1.0, 0.5 + pitch_input * 0.3 * acceleration_factor)
                
            except Exception as e:
                pass
    
    def update_physics(self):
        """Update flight physics"""
        # Process Bluetooth input
        self.process_bluetooth_input()
        
        # Smooth control surface movement
        roll_diff = self.target_roll - self.roll
        self.roll += roll_diff * 0.1 * 2.0
        
        pitch_diff = self.target_pitch - self.pitch
        self.pitch += pitch_diff * 0.08 * 1.0
        
        # Yaw from roll (banking turns)
        if abs(self.roll) > 5:
            yaw_change = (self.roll / 45.0) * 1.5
            self.yaw += yaw_change
        
        # Calculate forward direction
        yaw_rad = math.radians(self.yaw)
        pitch_rad = math.radians(self.pitch)
        
        forward = [
            math.sin(yaw_rad) * math.cos(pitch_rad),
            math.sin(pitch_rad),
            math.cos(yaw_rad) * math.cos(pitch_rad)
        ]
        
        # Thrust
        current_speed = math.sqrt(self.velocity[0]**2 + self.velocity[1]**2 + self.velocity[2]**2)
        thrust = self.throttle * self.acceleration
        
        # Apply thrust in forward direction
        for i in range(3):
            self.velocity[i] += forward[i] * thrust
        
        # Lift
        if current_speed > 0.1:
            lift_force = current_speed * self.lift_coefficient
            self.velocity[1] += lift_force
        
        # Gravity
        self.velocity[1] -= self.gravity
        
        # Drag
        for i in range(3):
            self.velocity[i] *= self.drag
        
        # Update position
        for i in range(3):
            self.position[i] += self.velocity[i]
        
        # Ground collision
        if self.position[1] < 1.0:
            self.position[1] = 1.0
            self.velocity[1] = max(0, self.velocity[1])
        
        # Update propeller
        self.propeller_rotation += self.propeller_speed * self.throttle
        self.propeller_rotation %= 360
    
    def update_camera(self):
        """Update camera position"""
        pos = self.position
        yaw_rad = math.radians(self.yaw)
        pitch_rad = math.radians(self.pitch)
        
        glLoadIdentity()
        
        # Chase camera
        distance = 30.0
        height = 10.0
        
        cam_x = pos[0] - math.sin(yaw_rad) * distance * math.cos(pitch_rad)
        cam_y = pos[1] + height - math.sin(pitch_rad) * distance * 0.5
        cam_z = pos[2] - math.cos(yaw_rad) * distance * math.cos(pitch_rad)
        
        look_x = pos[0] + math.sin(yaw_rad) * 10
        look_y = pos[1]
        look_z = pos[2] + math.cos(yaw_rad) * 10
        
        gluLookAt(cam_x, cam_y, cam_z, look_x, look_y, look_z, 0, 1, 0)
    
    def draw_terrain(self):
        """Draw ground terrain"""
        glDisable(GL_LIGHTING)
        glColor3f(0.2, 0.5, 0.2)  # Green
        
        # Ground plane
        glBegin(GL_QUADS)
        glVertex3f(-200, 0, -200)
        glVertex3f(200, 0, -200)
        glVertex3f(200, 0, 200)
        glVertex3f(-200, 0, 200)
        glEnd()
        
        # Grid lines
        glColor3f(0.3, 0.6, 0.3)
        glBegin(GL_LINES)
        for i in range(-20, 21):
            pos = i * 10
            # X lines
            glVertex3f(pos, 0.1, -200)
            glVertex3f(pos, 0.1, 200)
            # Z lines
            glVertex3f(-200, 0.1, pos)
            glVertex3f(200, 0.1, pos)
        glEnd()
        
        glEnable(GL_LIGHTING)
    
    def draw_airplane(self):
        """Draw the 3D airplane - FIXED to prevent stack overflow"""
        glPushMatrix()
        
        # Apply position
        glTranslatef(*self.position)
        
        # Apply rotations
        glRotatef(self.yaw, 0, 1, 0)
        glRotatef(self.pitch, 0, 0, 1)
        glRotatef(self.roll, 1, 0, 0)
        
        # Airplane colors
        colors = {
            'fuselage': (0.9, 0.9, 0.95),
            'wings': (0.85, 0.85, 0.9),
            'nose': (0.2, 0.3, 0.5),
            'tail': (0.9, 0.2, 0.2),
            'propeller': (0.3, 0.3, 0.35)
        }
        
        # Fuselage - FIX: Use reusable quadric
        glColor3f(*colors['fuselage'])
        glPushMatrix()
        glRotatef(90, 0, 1, 0)
        gluCylinder(self.quadric, 0.8, 0.8, 8.0, 16, 1)
        glPopMatrix()
        
        # Nose cone - FIX: Use reusable quadric
        glColor3f(*colors['nose'])
        glPushMatrix()
        glTranslatef(4.0, 0, 0)
        glRotatef(90, 0, 1, 0)
        gluCylinder(self.quadric, 0.8, 0, 1.5, 16, 1)
        glPopMatrix()
        
        # Wings
        glColor3f(*colors['wings'])
        glPushMatrix()
        glTranslatef(0, -0.3, 0)
        
        # Right wing
        glPushMatrix()
        glTranslatef(6.0, 0, 0)
        glScalef(6.0, 0.2, 2.5)
        self.draw_box(1, 1, 1)
        glPopMatrix()
        
        # Left wing
        glPushMatrix()
        glTranslatef(-6.0, 0, 0)
        glScalef(6.0, 0.2, 2.5)
        self.draw_box(1, 1, 1)
        glPopMatrix()
        
        glPopMatrix()
        
        # Tail
        glColor3f(*colors['tail'])
        glPushMatrix()
        glTranslatef(-3.5, 1.5, 0)
        glScalef(0.2, 3.0, 1.5)
        self.draw_box(1, 1, 1)
        glPopMatrix()
        
        # Propeller
        glPushMatrix()
        glTranslatef(4.5, 0, 0)
        
        # Propeller blades
        glColor3f(*colors['propeller'])
        glRotatef(self.propeller_rotation, 1, 0, 0)
        
        for i in range(3):
            glPushMatrix()
            glRotatef(i * 120, 1, 0, 0)
            glScalef(0.15, 2.0, 0.4)
            self.draw_box(1, 1, 1)
            glPopMatrix()
        
        glPopMatrix()
        
        glPopMatrix()
    
    def draw_box(self, w, h, d):
        """Draw a box"""
        glBegin(GL_QUADS)
        # Front
        glNormal3f(0, 0, 1)
        glVertex3f(-w, -h, d); glVertex3f(w, -h, d)
        glVertex3f(w, h, d); glVertex3f(-w, h, d)
        # Back
        glNormal3f(0, 0, -1)
        glVertex3f(-w, -h, -d); glVertex3f(-w, h, -d)
        glVertex3f(w, h, -d); glVertex3f(w, -h, -d)
        # Top
        glNormal3f(0, 1, 0)
        glVertex3f(-w, h, -d); glVertex3f(-w, h, d)
        glVertex3f(w, h, d); glVertex3f(w, h, -d)
        # Bottom
        glNormal3f(0, -1, 0)
        glVertex3f(-w, -h, -d); glVertex3f(w, -h, -d)
        glVertex3f(w, -h, d); glVertex3f(-w, -h, d)
        # Right
        glNormal3f(1, 0, 0)
        glVertex3f(w, -h, -d); glVertex3f(w, h, -d)
        glVertex3f(w, h, d); glVertex3f(w, -h, d)
        # Left
        glNormal3f(-1, 0, 0)
        glVertex3f(-w, -h, -d); glVertex3f(-w, -h, d)
        glVertex3f(-w, h, d); glVertex3f(-w, h, -d)
        glEnd()
    
    def draw_hud(self):
        """Draw heads-up display"""
        try:
            glDisable(GL_LIGHTING)
            glDisable(GL_DEPTH_TEST)
            
            glMatrixMode(GL_PROJECTION)
            glPushMatrix()
            glLoadIdentity()
            gluOrtho2D(0, self.display[0], self.display[1], 0)
            
            glMatrixMode(GL_MODELVIEW)
            glPushMatrix()
            glLoadIdentity()
            
            # HUD background
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            glColor4f(0.0, 0.0, 0.0, 0.7)
            glBegin(GL_QUADS)
            glVertex2f(10, 10)
            glVertex2f(350, 10)
            glVertex2f(350, 200)
            glVertex2f(10, 200)
            glEnd()
            
            # Flight data
            speed = math.sqrt(self.velocity[0]**2 + self.velocity[1]**2 + self.velocity[2]**2)
            altitude = self.position[1]
            
            bt_status = "Bluetooth" if self.bt_connected else "Keyboard"
            
            info = [
                f"Speed: {speed * 100:.0f} km/h",
                f"Altitude: {altitude:.1f} m",
                f"Throttle: {self.throttle * 100:.0f}%",
                f"Pitch: {self.pitch:.1f}°",
                f"Roll: {self.roll:.1f}°",
                f"Heading: {self.yaw % 360:.0f}°",
                f"Position: ({self.position[0]:.1f}, {self.position[1]:.1f}, {self.position[2]:.1f})",
                f"Control: {bt_status}",
                f"BT Data: X={self.bt_data['x']}, Y={self.bt_data['y']}",
                f"FPS: {self.current_fps:.0f}"
            ]
            
            # Draw text (simplified)
            glColor3f(1.0, 1.0, 1.0)
            for i, line in enumerate(info):
                # This is a placeholder - in a real implementation you'd use pygame.font
                pass
            
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            
            glEnable(GL_DEPTH_TEST)
            glEnable(GL_LIGHTING)
        
        except Exception as e:
            pass
    
    def handle_events(self):
        """Handle input events"""
        for event in pygame.event.get():
            if event.type == QUIT:
                self.running = False
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    self.running = False
                elif event.key == K_r:
                    self.reset_position()
                elif event.key == K_SPACE:
                    # Level flight
                    self.target_roll = 0
                    self.target_pitch = 0
        
        # Continuous keyboard input
        keys = pygame.key.get_pressed()
        
        # Pitch control
        if keys[K_UP]:
            self.target_pitch = 30.0
        elif keys[K_DOWN]:
            self.target_pitch = -30.0
        else:
            if not self.bt_connected:
                self.target_pitch = 0.0
        
        # Roll control
        if keys[K_LEFT]:
            self.target_roll = -45.0
        elif keys[K_RIGHT]:
            self.target_roll = 45.0
        else:
            if not self.bt_connected:
                self.target_roll = 0.0
        
        # Throttle control
        if keys[K_w]:
            self.throttle = min(1.0, self.throttle + 0.02)
        elif keys[K_s]:
            self.throttle = max(0.0, self.throttle - 0.02)
        
        # Manual yaw
        if keys[K_a]:
            self.yaw -= 2.0
        elif keys[K_d]:
            self.yaw += 2.0
    
    def reset_position(self):
        """Reset airplane to starting position"""
        self.position = [0.0, 20.0, -50.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll = 0.0
        self.throttle = 0.5
        self.target_roll = 0.0
        self.target_pitch = 0.0
    
    def update_fps(self):
        """Update FPS counter"""
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:
            self.current_fps = self.frame_count
            self.frame_count = 0
            self.last_fps_time = current_time
    
    def run(self):
        """Main game loop"""
        print("\n" + "="*60)
        print("3D Flight Simulator - Bluetooth Control - FIXED")
        print("="*60)
        print("Controls:")
        print("  Bluetooth Joystick: Primary control")
        print("  Arrow Keys: Pitch and Roll")
        print("  W/S: Throttle up/down")
        print("  A/D: Yaw left/right")
        print("  R: Reset position")
        print("  SPACE: Level flight")
        print("  ESC: Quit")
        print("="*60 + "\n")
        
        while self.running:
            # Handle events
            self.handle_events()
            
            # Update physics
            self.update_physics()
            
            # Render
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            
            # Set camera
            self.update_camera()
            
            # Draw scene
            self.draw_terrain()
            self.draw_airplane()
            
            # Draw HUD
            self.draw_hud()
            
            # Update display
            pygame.display.flip()
            self.clock.tick(self.fps)
            self.update_fps()
        
        # Cleanup
        if self.bt_controller:
            self.bt_controller.stop()
        
        # FIX: Delete quadric object on cleanup
        if self.quadric:
            gluDeleteQuadric(self.quadric)
        
        pygame.quit()

def main():
    """Main function"""
    try:
        print("🚀 Starting 3D Flight Simulator...")
        simulator = SimpleFlightSim()
        simulator.run()
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        input("\nPress Enter to exit...")

if __name__ == "__main__":
    main()