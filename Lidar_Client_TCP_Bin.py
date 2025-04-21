# Versão fullscreen (SFS) com leitura em thread separada

import os
import time
import pygame
import math
import socket
import struct
import csv
import sys
import argparse
import threading
from math import pi, sin, cos
from time import monotonic

TCP_PORT = 9999
GRID_FONT_SIZE = 10
UI_FONT_SIZE = 16

class LidarViewerSFS:
    def __init__(self, fullscreen=True, window_size=(800, 800), tcp_host="127.0.0.1", freq_hz=5):
        self.tcp_host = tcp_host
        self.freq_hz = freq_hz
        self.last_frame_time = 0
        self.datapack = []
        self.last_delay_ms = -1
        self.total_frames = 0
        self.running = True
        self.show_overlay = True
        self.scale = None
        self.dot_size = 3
        self.gridstyle = 0

        self.sock = None
        self.lock = threading.Lock()

        pygame.init()
        info = pygame.display.Info()
        self.display_size = (info.current_w, info.current_h) if fullscreen else window_size
        self.screen = pygame.display.set_mode(self.display_size, pygame.FULLSCREEN if fullscreen else 0)

        raw_ws = min(self.display_size)
        self.ws = raw_ws - (raw_ws % 16)
        self.hws = self.ws // 2
        self.center = ((self.display_size[0] - self.ws) // 2 + self.hws,
                       (self.display_size[1] - self.ws) // 2 + self.hws)
        self.grid_rect = pygame.Rect(self.center[0] - self.hws, self.center[1] - self.hws, self.ws, self.ws)

        self.max_scale = 8000 / self.hws
        self.scale = self.max_scale

        pygame.display.set_caption("Lidar Viewer SFS")
        pygame.font.init()
        self.grid_font = pygame.font.SysFont('Calibri', GRID_FONT_SIZE)
        self.ui_font = pygame.font.SysFont('Calibri', UI_FONT_SIZE)
        self.clock = pygame.time.Clock()

        self.set_dark_mode()
        self.grid_surface = self.create_grid()

        self.reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.reader_thread.start()

    def connect_socket(self):
        try:
            sock = socket.create_connection((self.tcp_host, TCP_PORT), timeout=1.0)
            sock.settimeout(1.0)
            return sock
        except Exception:
            return None

    def recv_all(self, sock, size):
        data = b''
        while len(data) < size:
            chunk = sock.recv(size - len(data))
            if not chunk:
                raise ConnectionError("Socket fechado inesperadamente")
            data += chunk
        return data

    def reader_loop(self):
        while self.running:
            now = monotonic()
            if now - self.last_frame_time < 1.0 / self.freq_hz:
                time.sleep(0.001)
                continue

            sock = self.connect_socket()
            if sock is None:
                time.sleep(0.2)
                continue

            try:
                header = self.recv_all(sock, 12)
                num_points, timestamp = struct.unpack('<IQ', header)
                frame_data = self.recv_all(sock, num_points * 6)
                points = [struct.unpack_from('<fH', frame_data, offset=i * 6) for i in range(num_points)]
                now_ms = int(time.time() * 1000)
            except Exception:
                sock.close()
                continue

            sock.close()

            with self.lock:
                self.last_frame_time = now
                self.last_delay_ms = now_ms - timestamp
                self.datapack = points
                self.total_frames += 1

    def set_dark_mode(self):
        self.backcolor = (0, 0, 0)
        self.gridcolor = (80, 80, 255)
        self.lessergridcolor = (0, 0, 96)
        self.gridcentercolor = (255, 255, 255)

    def set_light_mode(self):
        self.backcolor = (255, 255, 255)
        self.gridcolor = (0, 0, 128)
        self.lessergridcolor = (192, 192, 255)
        self.gridcentercolor = (0, 0, 0)

    def create_grid(self):
        surface = pygame.Surface(self.display_size).convert()
        surface.fill(self.backcolor)
        major = self.hws // 8
        minor = max(1, major // 5)

        if self.gridstyle == 0:
            for i in range(8):
                pygame.draw.circle(surface, self.gridcolor, self.center, (i + 1) * major, 1)
                for j in range(minor, major, minor):
                    pygame.draw.circle(surface, self.lessergridcolor, self.center, (i + 1) * major - j, 1)
            for i in range(24):
                angle = i * pi / 12
                x = self.center[0] + self.hws * cos(angle)
                y = self.center[1] - self.hws * sin(angle)
                color = self.gridcolor if i % 3 == 0 else self.lessergridcolor
                pygame.draw.line(surface, color, self.center, (int(x), int(y)), 1)
        else:
            for i in range(self.grid_rect.left, self.grid_rect.right, minor):
                pygame.draw.line(surface, self.lessergridcolor, (i, self.grid_rect.top), (i, self.grid_rect.bottom))
            for i in range(self.grid_rect.top, self.grid_rect.bottom, minor):
                pygame.draw.line(surface, self.lessergridcolor, (self.grid_rect.left, i), (self.grid_rect.right, i))
            for i in range(self.grid_rect.left, self.grid_rect.right, major):
                pygame.draw.line(surface, self.gridcolor, (i, self.grid_rect.top), (i, self.grid_rect.bottom))
            for i in range(self.grid_rect.top, self.grid_rect.bottom, major):
                pygame.draw.line(surface, self.gridcolor, (self.grid_rect.left, i), (self.grid_rect.right, i))

        pygame.draw.line(surface, self.gridcentercolor, (self.grid_rect.left, self.center[1]), (self.grid_rect.right, self.center[1]))
        pygame.draw.line(surface, self.gridcentercolor, (self.center[0], self.grid_rect.top), (self.center[0], self.grid_rect.bottom))

        for i in range(8):
            dist = (i + 1) * 1000 * (self.scale / self.max_scale)
            txt = self.grid_font.render(f"{int(dist)} mm", False, self.gridcolor)
            surface.blit(txt, (self.center[0] + 10, self.center[1] - ((i + 1) * major)))
            surface.blit(txt, (self.center[0] + 10, self.center[1] + ((i + 1) * major)))

        return surface

    def update_screen(self):
        self.screen.fill(self.backcolor)
        self.screen.blit(self.grid_surface, (0, 0))
        with self.lock:
            for polar in self.datapack:
                pt = self.polar_to_point(polar)
                pygame.draw.circle(self.screen, (255, 0, 0), (int(pt[0]), int(pt[1])), self.dot_size)

            if self.show_overlay:
                if self.last_delay_ms >= 0:
                    text = self.ui_font.render(f"Delay: {self.last_delay_ms:.1f} ms", False, (255, 255, 0))
                    self.screen.blit(text, (10, self.display_size[1] - 40))
                fps_text = self.ui_font.render(f"FPS: {self.clock.get_fps():.1f}", False, (0, 255, 0))
                self.screen.blit(fps_text, (10, self.display_size[1] - 20))
        pygame.display.update()

    def polar_to_point(self, polar):
        angle, dist = polar
        ang = angle * pi / 180.0
        dist /= self.scale
        return (self.center[0] - dist * sin(ang), self.center[1] + dist * cos(ang))

    def handle_key(self, key):
        if key in [pygame.K_KP_PLUS, pygame.K_EQUALS] and self.scale > 3:
            self.scale /= 2
        elif key in [pygame.K_KP_MINUS, pygame.K_MINUS] and self.scale < self.max_scale:
            self.scale *= 2
        elif key in [pygame.K_1, pygame.K_KP1]: self.scale = self.max_scale / 8
        elif key in [pygame.K_2, pygame.K_KP2]: self.scale = self.max_scale / 4
        elif key in [pygame.K_3, pygame.K_KP3]: self.scale = self.max_scale * 3 / 8
        elif key in [pygame.K_4, pygame.K_KP4]: self.scale = self.max_scale / 2
        elif key in [pygame.K_5, pygame.K_KP5]: self.scale = self.max_scale * 5 / 8
        elif key in [pygame.K_6, pygame.K_KP6]: self.scale = self.max_scale * 3 / 4
        elif key in [pygame.K_7, pygame.K_KP7]: self.scale = self.max_scale * 7 / 8
        elif key in [pygame.K_8, pygame.K_KP8]: self.scale = self.max_scale
        elif key == pygame.K_LEFTBRACKET and self.dot_size > 1:
            self.dot_size -= 1
        elif key == pygame.K_RIGHTBRACKET and self.dot_size < 6:
            self.dot_size += 1
        elif key == pygame.K_g:
            self.gridstyle = (self.gridstyle + 1) % 2
        elif key == pygame.K_d:
            self.set_dark_mode()
        elif key == pygame.K_b:
            self.set_light_mode()
        elif key == pygame.K_o:
            self.show_overlay = not self.show_overlay
        self.grid_surface = self.create_grid()

    def run(self):
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (
                    event.type == pygame.KEYDOWN and (event.key == pygame.K_ESCAPE or event.key == pygame.K_q)):
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    self.handle_key(event.key)

            self.update_screen()
            self.clock.tick(60)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Lidar Viewer SFS")
    parser.add_argument('-f', '--fullscreen', action='store_true', help='Tela cheia')
    parser.add_argument('-w', '--window', type=int, help='Tamanho da janela (ex: 800)', default=800)
    parser.add_argument('-H', '--host', type=str, help='Endereço do servidor TCP (default: 127.0.0.1)', default='127.0.0.1')
    parser.add_argument('--Fs', type=float, help='Frequência de aquisição em Hz (default: 5)', default=5.0)
    args = parser.parse_args()

    viewer = LidarViewerSFS(
        fullscreen=args.fullscreen,
        window_size=(args.window, args.window),
        tcp_host=args.host,
        freq_hz=args.Fs
    )
    viewer.run()
