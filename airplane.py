"""
RwandAir 3D Flight Simulator - Night Edition
Bluetooth Joystick Control Integration
Optimized Performance

Requirements:
    pip install pygame PyOpenGL PyOpenGL-accelerate pyserial numpy

Features:
    - RwandAir branded airplane
    - Night sky with stars
    - Realistic flight physics
    - Bluetooth joystick integration
    - Multiple camera views
    - Optimized rendering
"""

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import math
import numpy as np
import serial
import serial.tools.list_ports
import threading
import time
import re
from collections import deque
import random

class RwandAirPlane:
    """RwandAir branded airplane with enhanced visibility"""
    
    def __init__(self):
        # Position - start closer and visible
        self.position = np.array([0.0, 100.0, -50.0], dtype=np.float32)
        self.velocity = np.array([0.0, 0.0, -3.0], dtype=np.float32)
        
        # Rotation
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll = 0.0
        
        # Flight parameters
        self.throttle = 0.7
        self.max_speed = 8.0
        self.acceleration = 0.04
        self.lift_coefficient = 0.12
        self.drag = 0.98
        self.gravity = 0.025
        
        # Control
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.roll_speed = 2.5
        self.pitch_speed = 1.8
        self.yaw_from_roll = 1.8
        
        # Safety bounds
        self.max_position = np.array([5000.0, 1500.0, 5000.0], dtype=np.float32)
        self.min_position = np.array([-5000.0, 5.0, -5000.0], dtype=np.float32)
        
        # Larger dimensions for better visibility
        self.fuselage_length = 12.0
        self.fuselage_radius = 1.2
        self.wing_span = 16.0
        self.wing_chord = 4.0
        self.tail_height = 4.0
        
        # RwandAir colors (blue, white, green, yellow)
        self.colors = {
            'fuselage_top': (0.0, 0.4, 0.8),      # RwandAir blue
            'fuselage_bottom': (1.0, 1.0, 1.0),   # White
            'accent': (0.95, 0.8, 0.0),            # Yellow/gold
            'green': (0.0, 0.6, 0.2),              # Green stripe
            'windows': (0.2, 0.3, 0.5),            # Dark blue windows
            'wings': (0.9, 0.9, 0.95),             # Light gray wings
            'tail': (0.0, 0.4, 0.8),               # Blue tail
            'engine': (0.5, 0.5, 0.55)             # Engine gray
        }
        
        # Animation
        self.propeller_rotation = 0.0
        self.propeller_speed = 60.0
        
        # Lights
        self.lights_on = True
        self.light_blink = 0
        
    def set_throttle(self, value):
        self.throttle = max(0.0, min(1.0, float(value)))
    
    def set_pitch_input(self, value):
        self.target_pitch = max(-60.0, min(60.0, float(value) * 35.0))
    
    def set_roll_input(self, value):
        self.target_roll = max(-60.0, min(60.0, float(value) * 50.0))
    
    def draw_cylinder(self, radius, height, slices=12):
        """Optimized cylinder"""
        glBegin(GL_TRIANGLE_STRIP)
        for i in range(slices + 1):
            angle = 2 * math.pi * i / slices
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            glNormal3f(x / radius, y / radius, 0)
            glVertex3f(x, y, 0)
            glVertex3f(x, y, height)
        glEnd()
        
        # Caps
        glBegin(GL_TRIANGLE_FAN)
        glNormal3f(0, 0, -1)
        glVertex3f(0, 0, 0)
        for i in range(slices + 1):
            angle = 2 * math.pi * i / slices
            glVertex3f(radius * math.cos(angle), radius * math.sin(angle), 0)
        glEnd()
        
        glBegin(GL_TRIANGLE_FAN)
        glNormal3f(0, 0, 1)
        glVertex3f(0, 0, height)
        for i in range(slices + 1):
            angle = 2 * math.pi * i / slices
            glVertex3f(radius * math.cos(angle), radius * math.sin(angle), height)
        glEnd()
    
    def draw_box(self, w, h, d):
        """Optimized box"""
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
    
    def draw_fuselage(self):
        """RwandAir branded fuselage"""
        # Main body - blue top
        glColor3f(*self.colors['fuselage_top'])
        glPushMatrix()
        glRotatef(90, 0, 1, 0)
        self.draw_cylinder(self.fuselage_radius, self.fuselage_length, 16)
        glPopMatrix()
        
        # White bottom stripe
        glColor3f(*self.colors['fuselage_bottom'])
        glPushMatrix()
        glTranslatef(0, -self.fuselage_radius * 0.6, 0)
        glRotatef(90, 0, 1, 0)
        self.draw_cylinder(self.fuselage_radius * 0.8, self.fuselage_length, 16)
        glPopMatrix()
        
        # Yellow accent stripe
        glColor3f(*self.colors['accent'])
        glPushMatrix()
        glTranslatef(0, 0, 0)
        glScalef(self.fuselage_length, 0.15, 0.15)
        self.draw_box(0.5, 1, 1)
        glPopMatrix()
        
        # Green stripe
        glColor3f(*self.colors['green'])
        glPushMatrix()
        glTranslatef(0, -self.fuselage_radius * 0.3, 0)
        glScalef(self.fuselage_length, 0.1, 0.1)
        self.draw_box(0.5, 1, 1)
        glPopMatrix()
        
        # Nose cone
        glPushMatrix()
        glTranslatef(self.fuselage_length * 0.5, 0, 0)
        glRotatef(90, 0, 1, 0)
        glColor3f(*self.colors['fuselage_top'])
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(0, 0, 1.5)
        for i in range(17):
            angle = 2 * math.pi * i / 16
            glVertex3f(self.fuselage_radius * math.cos(angle),
                      self.fuselage_radius * math.sin(angle), 0)
        glEnd()
        glPopMatrix()
    
    def draw_wings(self):
        """Large visible wings"""
        glColor3f(*self.colors['wings'])
        
        # Main wings with blue tips
        for side in [-1, 1]:
            glPushMatrix()
            glTranslatef(side * self.wing_span * 0.3, -0.3, 0)
            glScalef(self.wing_span * 0.35, 0.2, self.wing_chord)
            self.draw_box(1, 1, 1)
            glPopMatrix()
            
            # Wing tip - blue
            glColor3f(*self.colors['tail'])
            glPushMatrix()
            glTranslatef(side * self.wing_span * 0.48, -0.3, 0)
            glScalef(self.wing_span * 0.08, 0.18, self.wing_chord * 0.6)
            self.draw_box(1, 1, 1)
            glPopMatrix()
            glColor3f(*self.colors['wings'])
    
    def draw_engines(self):
        """Engine nacelles under wings"""
        glColor3f(*self.colors['engine'])
        
        for side in [-1, 1]:
            glPushMatrix()
            glTranslatef(side * self.wing_span * 0.25, -1.5, 0.5)
            glRotatef(90, 0, 1, 0)
            self.draw_cylinder(0.6, 2.5, 12)
            glPopMatrix()
    
    def draw_tail(self):
        """RwandAir tail with logo colors"""
        tail_x = -self.fuselage_length * 0.42
        
        # Vertical stabilizer - blue with accent
        glColor3f(*self.colors['tail'])
        glPushMatrix()
        glTranslatef(tail_x, self.tail_height * 0.35, 0)
        glScalef(0.2, self.tail_height * 0.7, 1.2)
        self.draw_box(1, 1, 1)
        glPopMatrix()
        
        # Yellow accent on tail
        glColor3f(*self.colors['accent'])
        glPushMatrix()
        glTranslatef(tail_x, self.tail_height * 0.6, 0)
        glScalef(0.22, self.tail_height * 0.15, 1.1)
        self.draw_box(1, 1, 1)
        glPopMatrix()
        
        # Horizontal stabilizer
        glColor3f(*self.colors['wings'])
        glPushMatrix()
        glTranslatef(tail_x, 0.5, 0)
        glScalef(2.2, 0.15, 1.0)
        self.draw_box(1, 1, 1)
        glPopMatrix()
    
    def draw_windows(self):
        """Cockpit and cabin windows"""
        glColor3f(*self.colors['windows'])
        
        # Cockpit windshield
        glPushMatrix()
        glTranslatef(self.fuselage_length * 0.25, 0.3, 0)
        glScalef(0.8, 0.4, 0.85)
        self.draw_box(1, 1, 1)
        glPopMatrix()
        
        # Cabin windows (simplified as stripes)
        glPushMatrix()
        glTranslatef(0, 0.3, 0)
        for i in range(6):
            x_pos = self.fuselage_length * 0.1 - i * 1.5
            glPushMatrix()
            glTranslatef(x_pos, 0, 0)
            glScalef(0.4, 0.3, 0.86)
            self.draw_box(1, 1, 1)
            glPopMatrix()
        glPopMatrix()
    
    def draw_navigation_lights(self):
        """Blinking navigation lights"""
        if not self.lights_on:
            return
        
        glDisable(GL_LIGHTING)
        
        # Red light - left wing
        if self.light_blink % 30 < 15:
            glColor3f(1.0, 0.0, 0.0)
            glPushMatrix()
            glTranslatef(-self.wing_span * 0.5, -0.3, 0)
            quadric = gluNewQuadric()
            gluSphere(quadric, 0.3, 8, 8)
            gluDeleteQuadric(quadric)
            glPopMatrix()
        
        # Green light - right wing
        if (self.light_blink + 15) % 30 < 15:
            glColor3f(0.0, 1.0, 0.0)
            glPushMatrix()
            glTranslatef(self.wing_span * 0.5, -0.3, 0)
            quadric = gluNewQuadric()
            gluSphere(quadric, 0.3, 8, 8)
            gluDeleteQuadric(quadric)
            glPopMatrix()
        
        glEnable(GL_LIGHTING)
        self.light_blink += 1
    
    def update_physics(self):
        """Update flight physics"""
        try:
            # Smooth controls
            self.roll += (self.target_roll - self.roll) * 0.12
            self.pitch += (self.target_pitch - self.pitch) * 0.1
            
            # Banking turns
            if abs(self.roll) > 5:
                self.yaw += (self.roll / 45.0) * self.yaw_from_roll
                self.yaw = self.yaw % 360.0
            
            # Forward direction
            yaw_rad = math.radians(self.yaw)
            pitch_rad = math.radians(self.pitch)
            
            forward = np.array([
                math.sin(yaw_rad) * math.cos(pitch_rad),
                math.sin(pitch_rad),
                math.cos(yaw_rad) * math.cos(pitch_rad)
            ], dtype=np.float32)
            
            # Thrust
            thrust = self.throttle * self.acceleration
            self.velocity += forward * thrust
            
            # Lift
            speed = np.linalg.norm(self.velocity)
            if speed > 0.1:
                lift = speed * self.lift_coefficient * (1.0 + abs(self.pitch) / 40.0)
                self.velocity[1] += lift
            
            # Gravity and drag
            self.velocity[1] -= self.gravity
            self.velocity *= self.drag
            
            # Update position
            self.position += self.velocity
            
            # Clamp position
            self.position = np.clip(self.position, self.min_position, self.max_position)
            
            # Ground collision
            if self.position[1] < 5.0:
                self.position[1] = 5.0
                self.velocity[1] = max(0, self.velocity[1])
            
            # Propeller
            self.propeller_rotation = (self.propeller_rotation + 
                                      self.propeller_speed * self.throttle) % 360.0
            
        except Exception as e:
            print(f"Physics error: {e}")
    
    def reset(self):
        """Reset to start"""
        self.position = np.array([0.0, 100.0, -50.0], dtype=np.float32)
        self.velocity = np.array([0.0, 0.0, -3.0], dtype=np.float32)
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll = 0.0
        self.throttle = 0.7
    
    def draw(self):
        """Draw complete airplane"""
        glPushMatrix()
        
        # Position and orientation
        glTranslatef(*self.position)
        glRotatef(self.yaw, 0, 1, 0)
        glRotatef(self.pitch, 0, 0, 1)
        glRotatef(self.roll, 1, 0, 0)
        
        # Draw components
        self.draw_fuselage()
        self.draw_wings()
        self.draw_engines()
        self.draw_tail()
        self.draw_windows()
        self.draw_navigation_lights()
        
        glPopMatrix()


class NightSky:
    """Night sky with stars"""
    
    def __init__(self):
        self.stars = []
        self.generate_stars()
    
    def generate_stars(self):
        """Generate star field"""
        random.seed(42)
        for _ in range(500):
            # Distribute stars in a large sphere
            theta = random.uniform(0, 2 * math.pi)
            phi = random.uniform(0, math.pi)
            distance = random.uniform(800, 1200)
            
            x = distance * math.sin(phi) * math.cos(theta)
            y = distance * math.cos(phi)
            z = distance * math.sin(phi) * math.sin(theta)
            
            # Keep stars above horizon
            if y > 0:
                brightness = random.uniform(0.3, 1.0)
                size = random.uniform(0.5, 2.0)
                self.stars.append((x, y, z, brightness, size))
    
    def draw(self):
        """Draw stars"""
        glDisable(GL_LIGHTING)
        glPointSize(2.0)
        
        glBegin(GL_POINTS)
        for star in self.stars:
            x, y, z, brightness, size = star
            glColor3f(brightness, brightness, brightness * 0.95)
            glVertex3f(x, y, z)
        glEnd()
        
        glEnable(GL_LIGHTING)


class OptimizedGround:
    """Optimized ground plane"""
    
    def __init__(self):
        self.size = 2000
        self.display_list = None
    
    def build_display_list(self):
        """Build ground mesh once"""
        if self.display_list is None:
            self.display_list = glGenLists(1)
        
        glNewList(self.display_list, GL_COMPILE)
        
        glDisable(GL_LIGHTING)
        
        # Dark ground for night
        glColor3f(0.05, 0.1, 0.05)
        glBegin(GL_QUADS)
        glVertex3f(-self.size/2, 0, -self.size/2)
        glVertex3f(self.size/2, 0, -self.size/2)
        glVertex3f(self.size/2, 0, self.size/2)
        glVertex3f(-self.size/2, 0, self.size/2)
        glEnd()
        
        # Grid lines
        glColor3f(0.15, 0.2, 0.15)
        glBegin(GL_LINES)
        step = 100
        for i in range(-self.size//2, self.size//2 + 1, step):
            glVertex3f(i, 0.1, -self.size/2)
            glVertex3f(i, 0.1, self.size/2)
            glVertex3f(-self.size/2, 0.1, i)
            glVertex3f(self.size/2, 0.1, i)
        glEnd()
        
        glEnable(GL_LIGHTING)
        
        glEndList()
    
    def draw(self):
        """Draw ground"""
        if self.display_list is None:
            self.build_display_list()
        glCallList(self.display_list)


DATA_PATTERN = re.compile(r"X:(\d+),Y:(\d+)")

class BluetoothController:
    """Bluetooth joystick controller"""
    
    def __init__(self):
        self.serial_conn = None
        self.connected = False
        self.running = False
        self.data_thread = None
        
        self.raw_data = {'x': 512, 'y': 512}
        self.smoothed_data = {'x': 512, 'y': 512}
        self.data_history = deque(maxlen=8)
        
        self.center_x = 512
        self.center_y = 512
        self.deadzone = 60
        self.max_deviation = 200

    def list_com_ports(self):
        return [port.device for port in serial.tools.list_ports.comports()]

    def test_com_port(self, port, baudrate=38400, timeout=1, max_lines=5):
        try:
            with serial.Serial(port, baudrate=baudrate, timeout=timeout) as ser:
                for _ in range(max_lines):
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if DATA_PATTERN.match(line):
                        return True
            return False
        except:
            return False

    def connect(self):
        print("Scanning for Bluetooth...")
        for port in self.list_com_ports():
            print(f"Testing {port}...")
            if self.test_com_port(port):
                try:
                    self.serial_conn = serial.Serial(port, 38400, timeout=1)
                    self.connected = True
                    print(f"✅ Connected to {port}")
                    return True
                except:
                    pass
        print("❌ No device found")
        return False
    
    def start_listening(self):
        if not self.connected:
            return False
        self.running = True
        self.data_thread = threading.Thread(target=self._data_loop, daemon=True)
        self.data_thread.start()
        return True
    
    def stop(self):
        self.running = False
        if self.data_thread:
            self.data_thread.join(timeout=1.0)
        if self.serial_conn:
            self.serial_conn.close()
    
    def _data_loop(self):
        try:
            while self.running and self.serial_conn:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    match = DATA_PATTERN.match(line)
                    if match:
                        x, y = map(int, match.groups())
                        self._process_data(x, y)
        except Exception as e:
            print(f"BT error: {e}")
            self.connected = False
    
    def _process_data(self, x, y):
        self.raw_data = {'x': x, 'y': y}
        
        # Apply deadzone
        x_diff = x - self.center_x
        y_diff = y - self.center_y
        
        if abs(x_diff) < self.deadzone:
            x_diff = 0
        if abs(y_diff) < self.deadzone:
            y_diff = 0
        
        # Normalize
        x_norm = np.clip(x_diff / self.max_deviation, -1.0, 1.0)
        y_norm = np.clip(-y_diff / self.max_deviation, -1.0, 1.0)
        
        # Smooth
        alpha = 0.3
        self.smoothed_data['x'] = alpha * x_norm + (1 - alpha) * self.smoothed_data['x']
        self.smoothed_data['y'] = alpha * y_norm + (1 - alpha) * self.smoothed_data['y']
    
    def get_control_inputs(self):
        return {
            'roll': self.smoothed_data['x'],
            'pitch': self.smoothed_data['y'],
            'throttle': max(0.0, min(1.0, 0.5 + self.smoothed_data['y'] * 0.3))
        }


class FlightSimulator:
    """Main simulator"""
    
    def __init__(self):
        pygame.init()
        self.display = (1366, 768)
        
        # OpenGL setup
        pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLEBUFFERS, 1)
        pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLESAMPLES, 4)
        
        self.screen = pygame.display.set_mode(self.display, DOUBLEBUF | OPENGL)
        pygame.display.set_caption("RwandAir Flight Simulator - Night Edition")
        
        # OpenGL settings
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_MULTISAMPLE)
        
        # Night sky background
        glClearColor(0.02, 0.02, 0.15, 1.0)  # Dark blue night
        
        # Moon lighting
        glLightfv(GL_LIGHT0, GL_POSITION, (200, 300, 100, 1))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.2, 0.2, 0.25, 1))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.6, 0.6, 0.7, 1))
        
        # Projection
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(65, self.display[0] / self.display[1], 1.0, 2500.0)
        glMatrixMode(GL_MODELVIEW)
        
        # Objects
        self.airplane = RwandAirPlane()
        self.ground = OptimizedGround()
        self.sky = NightSky()
        self.bt_controller = BluetoothController()
        
        # State
        self.clock = pygame.time.Clock()
        self.running = True
        self.font = pygame.font.Font(None, 28)
        
        # Camera
        self.camera_modes = ['chase', 'cockpit', 'side', 'orbit']
        self.camera_index = 0
        self.camera_distance = 30.0
    
    def update_camera(self):
        """Camera system"""
        mode = self.camera_modes[self.camera_index]
        pos = self.airplane.position
        yaw_rad = math.radians(self.airplane.yaw)
        pitch_rad = math.radians(self.airplane.pitch)
        
        glLoadIdentity()
        
        if mode == 'chase':
            # Follow behind
            dist = 35.0
            height = 12.0
            
            cam_x = pos[0] - math.sin(yaw_rad) * dist
            cam_y = pos[1] + height
            cam_z = pos[2] - math.cos(yaw_rad) * dist
            
            look_x = pos[0] + math.sin(yaw_rad) * 15
            look_y = pos[1]
            look_z = pos[2] + math.cos(yaw_rad) * 15
            
            gluLookAt(cam_x, cam_y, cam_z, look_x, look_y, look_z, 0, 1, 0)
        
        elif mode == 'cockpit':
            # First person
            cam_x = pos[0] + math.sin(yaw_rad) * 3
            cam_y = pos[1] + 2
            cam_z = pos[2] + math.cos(yaw_rad) * 3
            
            look_x = pos[0] + math.sin(yaw_rad) * 100
            look_y = pos[1] + math.sin(pitch_rad) * 100
            look_z = pos[2] + math.cos(yaw_rad) * 100
            
            roll_rad = math.radians(self.airplane.roll)
            gluLookAt(cam_x, cam_y, cam_z, look_x, look_y, look_z,
                     math.sin(roll_rad), math.cos(roll_rad), 0)
        
        elif mode == 'side':
            # Side view
            cam_x = pos[0] + 50
            cam_y = pos[1] + 20
            cam_z = pos[2]
            gluLookAt(cam_x, cam_y, cam_z, pos[0], pos[1], pos[2], 0, 1, 0)
        
        else:  # orbit
            # Orbiting view
            angle = pygame.time.get_ticks() * 0.0002
            cam_x = pos[0] + math.cos(angle) * 60
            cam_y = pos[1] + 25
            cam_z = pos[2] + math.sin(angle) * 60
            gluLookAt(cam_x, cam_y, cam_z, pos[0], pos[1], pos[2], 0, 1, 0)
    
    def draw_hud(self):
        """HUD display"""
        glDisable(GL_LIGHTING)
        glDisable(GL_DEPTH_TEST)
        
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        gluOrtho2D(0, self.display[0], self.display[1], 0)
        
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        
        # Panel background
        panel_w, panel_h = 400, 240
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glColor4f(0.0, 0.2, 0.4, 0.8)
        glBegin(GL_QUADS)
        glVertex2f(10, 10)
        glVertex2f(10 + panel_w, 10)
        glVertex2f(10 + panel_w, 10 + panel_h)
        glVertex2f(10, 10 + panel_h)
        glEnd()
        
        # RwandAir logo colors accent
        glColor4f(0.95, 0.8, 0.0, 0.9)
        glBegin(GL_QUADS)
        glVertex2f(10, 10)
        glVertex2f(10 + panel_w, 10)
        glVertex2f(10 + panel_w, 15)
        glVertex2f(10, 15)
        glEnd()
        
        # Stats
        speed = np.linalg.norm(self.airplane.velocity) * 100
        altitude = self.airplane.position[1]
        throttle = self.airplane.throttle * 100
        
        bt_status = "🔵 BLUETOOTH" if self.bt_controller.connected else "⌨️ KEYBOARD"
        cam_name = self.camera_modes[self.camera_index].upper()
        
        info = [
            "╔═══ RWANDAIR FLIGHT ═══╗",
            f"Speed:    {speed:.0f} km/h",
            f"Altitude: {altitude:.0f} m",
            f"Throttle: {throttle:.0f}%",
            f"Pitch:    {self.airplane.pitch:.1f}°",
            f"Roll:     {self.airplane.roll:.1f}°",
            f"Heading:  {self.airplane.yaw % 360:.0f}°",
            f"Camera:   {cam_name}",
            f"Control:  {bt_status}",
            "╚═══════════════════════╝"
        ]
        
        for i, line in enumerate(info):
            text = self.font.render(line, True, (255, 255, 255))
            text_data = pygame.image.tostring(text, "RGBA", True)
            glRasterPos2i(20, 30 + i * 24)
            glDrawPixels(text.get_width(), text.get_height(),
                        GL_RGBA, GL_UNSIGNED_BYTE, text_data)
        
        glDisable(GL_BLEND)
        
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
    
    def run(self):
        """Main loop"""
        # Connect Bluetooth
        if not self.bt_controller.connect():
            print("⚠️  Keyboard mode")
        else:
            self.bt_controller.start_listening()
            print("✅ Bluetooth active!")
        
        print("\n" + "="*60)
        print("🇷🇼  RWANDAIR NIGHT FLIGHT SIMULATOR")
        print("="*60)
        print("\nControls:")
        print("  Joystick: Roll & Pitch")
        print("  Keyboard: Arrow Keys, W/S (throttle)")
        print("  C: Change camera")
        print("  R: Reset")
        print("  L: Toggle lights")
        print("  ESC: Quit")
        print("="*60 + "\n")
        
        while self.running:
            # Events
            for event in pygame.event.get():
                if event.type == QUIT:
                    self.running = False
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        self.running = False
                    elif event.key == K_c:
                        self.camera_index = (self.camera_index + 1) % len(self.camera_modes)
                        print(f"Camera: {self.camera_modes[self.camera_index]}")
                    elif event.key == K_r:
                        self.airplane.reset()
                        print("Reset")
                    elif event.key == K_l:
                        self.airplane.lights_on = not self.airplane.lights_on
                        print(f"Lights: {'ON' if self.airplane.lights_on else 'OFF'}")
                    elif event.key == K_SPACE:
                        self.airplane.target_roll = 0
                        self.airplane.target_pitch = 0
            
            # Keyboard input
            keys = pygame.key.get_pressed()
            
            if keys[K_UP]:
                self.airplane.set_pitch_input(1.0)
            elif keys[K_DOWN]:
                self.airplane.set_pitch_input(-1.0)
            
            if keys[K_LEFT]:
                self.airplane.set_roll_input(-1.0)
            elif keys[K_RIGHT]:
                self.airplane.set_roll_input(1.0)
            
            if keys[K_w]:
                self.airplane.set_throttle(min(1.0, self.airplane.throttle + 0.01))
            elif keys[K_s]:
                self.airplane.set_throttle(max(0.0, self.airplane.throttle - 0.01))
            
            # Bluetooth input
            if self.bt_controller.connected:
                controls = self.bt_controller.get_control_inputs()
                self.airplane.set_roll_input(controls['roll'])
                self.airplane.set_pitch_input(controls['pitch'])
                self.airplane.set_throttle(controls['throttle'])
            
            # Update
            self.airplane.update_physics()
            
            # Render
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            
            self.update_camera()
            
            # Draw scene
            self.sky.draw()
            self.ground.draw()
            self.airplane.draw()
            
            self.draw_hud()
            
            pygame.display.flip()
            self.clock.tick(60)
        
        # Cleanup
        self.bt_controller.stop()
        pygame.quit()


if __name__ == '__main__':
    try:
        print("\n🇷🇼  RwandAir Flight Simulator - Night Edition")
        print("Initializing...\n")
        
        sim = FlightSimulator()
        sim.run()
        
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        input("\nPress Enter to exit...")