#!/usr/bin/env python3
"""
Tkinter GUI for rec_rep2

Usage: launch via `ros2 run rec_rep2 gui`.
"""

import os, signal, subprocess, threading
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

# ── Paths ─────────────────────────────────────────────────────────────────────
BAGS_DIR    = os.path.expanduser('~/ros2_ws/src/rec_rep2/bags')
WS_OVERLAY  = os.path.expanduser('~/ros2_ws/install/setup.bash')
SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ── Theme ─────────────────────────────────────────────────────────────────────
BG, BG_S, BG_I, BDR = '#f8f7f5', '#ffffff', '#f1efe8', '#d3d1c7'
FG, FG2, FGM        = '#1a1a1a', '#5f5e5a', '#888780'
C_OK, C_WARN, C_ERR = '#3b6d11', '#854f0b', '#a32d2d'
F_OK, F_WARN, F_ERR = '#eaf3de', '#faeeda', '#fcebeb'

FH = ('Helvetica', 13, 'bold')
FB = ('Helvetica', 10)
FL = ('Helvetica', 9)
FM = ('Courier', 9)

SEMANTIC = {
    'success':   (C_OK,   F_OK),
    'warning':   (C_WARN, F_WARN),
    'danger':    (C_ERR,  F_ERR),
    'recording': (C_ERR,  F_ERR),
    'neutral':   (FGM,    BG_I),
}

# ── Fake-hardware paths ───────────────────────────────────────────────────────
XACRO_PATH = '/opt/ros/humble/share/kortex_description/robots/gen3.xacro'
XACRO_ARGS = 'robot_ip:=0.0.0.0 name:=arm arm:=gen3 dof:=7 use_fake_hardware:=true'
URDF_TMP   = '/tmp/rec_rep2_gen3.urdf'
RVIZ_CFG   = '/opt/ros/humble/share/kortex_description/rviz/view_robot.rviz'


# ── ROS2 node ─────────────────────────────────────────────────────────────────
class GuiNode(Node):
    def __init__(self):
        super().__init__('rec_rep2_gui')
        self._clis = {
            'start': self.create_client(Trigger, '/motion_recorder/start_recording'),
            'stop':  self.create_client(Trigger, '/motion_recorder/stop_recording'),
            'mode':  self.create_client(SetBool, '/motion_recorder/set_posing_mode'),
            'bias':  self.create_client(Trigger, '/motion_recorder/confirm_bias_zeroed'),
            'fault': self.create_client(Trigger, '/motion_recorder/reset_fault'),
        }
        self._lock = threading.Lock()

    def recorder_available(self):
        return self._clis['start'].service_is_ready()

    def _call_trigger(self, key):
        cli = self._clis[key]
        with self._lock:
            if not cli.wait_for_service(timeout_sec=2.0):
                return False, 'Recorder node not reachable'
            fut = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result()
        return r.success, r.message

    def call_start(self):        return self._call_trigger('start')
    def call_stop(self):         return self._call_trigger('stop')
    def call_confirm_bias(self): return self._call_trigger('bias')
    def call_reset_fault(self):  return self._call_trigger('fault')

    def call_set_mode(self, use_torque: bool):
        cli = self._clis['mode']
        with self._lock:
            if not cli.wait_for_service(timeout_sec=2.0):
                return False, 'Recorder node not reachable'
            req = SetBool.Request(); req.data = use_torque
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result()
        return r.success, r.message


# ── App ───────────────────────────────────────────────────────────────────────
class App(tk.Tk):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self._last_file = None
        self._recording = False
        self._fake_mode = False
        self._procs = {}  # name → Popen

        self.title('rec_rep2')
        self.configure(bg=BG, padx=18, pady=18)
        self.minsize(520, 600)
        self.protocol('WM_DELETE_WINDOW', self._on_close)
        self._build_ui()
        self._poll_status()

    # ── Widget helpers ────────────────────────────────────────────────────────

    def _card(self, parent):
        return tk.Frame(parent, bg=BG_S,
                        highlightbackground=BDR, highlightthickness=1,
                        padx=10, pady=10)

    def _hlabel(self, parent, text):
        return tk.Label(parent, text=text.upper(),
                        font=('Helvetica', 8, 'bold'),
                        bg=BG_S, fg=FGM, anchor='w')

    def _btn(self, parent, text, cmd, style='primary', width=18, state='normal'):
        styles = {
            'primary': dict(bg=FG,   fg='#fff', relief='flat', bd=0,
                            activebackground='#333'),
            'danger':  dict(bg=BG_S, fg=C_ERR,  relief='solid', bd=1,
                            activebackground=F_ERR),
            'ghost':   dict(bg=BG_S, fg=FG2,    relief='solid', bd=1,
                            activebackground=BG_I),
        }
        kw = styles.get(style, styles['ghost'])
        abg = kw.pop('activebackground')
        b = tk.Button(parent, text=text, command=cmd, font=FB,
                      activebackground=abg, activeforeground=kw['fg'],
                      disabledforeground=FGM, width=width, state=state,
                      padx=8, pady=5, **kw)
        if style != 'primary':
            b.config(highlightbackground=BDR, highlightthickness=1)
        return b

    # ── Process helpers ───────────────────────────────────────────────────────

    def _ros_cmd(self, parts):
        chain = ['source /opt/ros/humble/setup.bash',
                 f'source {WS_OVERLAY}'] + parts
        return ['bash', '-c', ' && '.join(chain)]

    def _spawn(self, name, ros_args, sem='neutral'):
        p = subprocess.Popen(self._ros_cmd(ros_args),
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             preexec_fn=os.setsid)
        self._procs[name] = p
        self._log(f'Launched {name} (pid {p.pid})', sem)
        threading.Thread(target=self._tail, args=(p, name), daemon=True).start()

    def _kill(self, name):
        p = self._procs.pop(name, None)
        if p and p.poll() is None:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            self._log(f'{name} stopped.', 'danger')

    def _tail(self, proc, prefix):
        for raw in proc.stdout:
            self._log(f'[{prefix}] {raw.decode(errors="replace").rstrip()}')

    def _log(self, msg, sem='neutral'):
        color = SEMANTIC.get(sem, (FG2, BG_I))[0]
        def _do():
            tag = f't_{sem}'
            self._logw.config(state='normal')
            self._logw.insert('end', msg + '\n', tag)
            self._logw.tag_config(tag, foreground=color)
            self._logw.see('end')
            self._logw.config(state='disabled')
        self.after(0, _do)

    def _bg_call(self, fn, *args, on_ok=None):
        """Run fn(*args) in a thread; log result; call on_ok() on success."""
        def _w():
            ok, msg = fn(*args)
            self._log(('[OK] ' if ok else '[FAIL] ') + msg,
                      'success' if ok else 'danger')
            if ok and on_ok:
                self.after(0, on_ok)
        threading.Thread(target=_w, daemon=True).start()

    # ── UI build ──────────────────────────────────────────────────────────────

    def _build_ui(self):
        tk.Label(self, text='Motion Record & Replay',
                 font=FH, bg=BG, fg=FG, anchor='w').pack(fill='x', pady=(0, 12))
        self._build_status()
        self._build_launch()
        self._build_record()
        self._build_replay()
        self._build_log()

    # ── Status card ───────────────────────────────────────────────────────────

    def _build_status(self):
        card = self._card(self); card.pack(fill='x', pady=(0, 8))
        self._hlabel(card, 'System status').pack(fill='x', pady=(0, 6))

        self._sv = {}
        for key, label in [('recorder', 'Recorder node'),
                            ('robot',    'Robot (gRPC)'),
                            ('state',    'Recording state')]:
            row = tk.Frame(card, bg=BG_S); row.pack(fill='x', pady=2)
            tk.Label(row, text=label, width=16, anchor='w',
                     bg=BG_S, fg=FG2, font=FL).pack(side='left')
            dot = tk.Label(row, text='  ', bg=BG_I, fg=FGM,
                           font=FL, padx=8, pady=2)
            dot.pack(side='left')
            var = tk.StringVar(value='...')
            lbl = tk.Label(row, textvariable=var, anchor='w',
                           bg=BG_S, fg=FGM, font=FL)
            lbl.pack(side='left', padx=(4, 0))
            self._sv[key] = (var, lbl, dot)

        tk.Frame(card, bg=BDR, height=1).pack(fill='x', pady=(8, 6))
        row = tk.Frame(card, bg=BG_S); row.pack(fill='x')
        tk.Label(row, text='Last saved', width=16, anchor='w',
                 bg=BG_S, fg=FG2, font=FL).pack(side='left')
        self._last_var = tk.StringVar(value='—')
        tk.Label(row, textvariable=self._last_var, anchor='w',
                 bg=BG_S, fg=FGM, font=FM, wraplength=300).pack(side='left')

    def _set_status(self, key, text, sem):
        fg, bg = SEMANTIC.get(sem, (FGM, BG_I))
        var, lbl, dot = self._sv[key]
        var.set(text); lbl.config(fg=fg); dot.config(bg=bg, fg=fg)

    # ── Launch card ───────────────────────────────────────────────────────────

    def _build_launch(self):
        card = self._card(self); card.pack(fill='x', pady=(0, 8))
        self._hlabel(card, 'Node launch').pack(fill='x', pady=(0, 8))

        row = tk.Frame(card, bg=BG_S); row.pack(fill='x', pady=(0, 8))
        tk.Label(row, text='Robot IP', width=10, anchor='w',
                 bg=BG_S, fg=FG2, font=FL).pack(side='left')
        self._ip_var = tk.StringVar(value='192.168.0.10')
        tk.Entry(row, textvariable=self._ip_var, width=18,
                 bg=BG_I, fg=FG, insertbackground=FG,
                 relief='solid', bd=1, font=FB).pack(side='left', padx=(6, 0))

        self._fake_var = tk.BooleanVar(value=False)
        tk.Checkbutton(card, text='Fake hardware (no robot required)',
                       variable=self._fake_var, bg=BG_S, fg=FG2,
                       activebackground=BG_S, selectcolor=BG_I,
                       font=FL, bd=0, highlightthickness=0).pack(anchor='w', pady=(0, 8))

        row = tk.Frame(card, bg=BG_S); row.pack(fill='x', pady=(0, 6))
        self._launch_btn = self._btn(row, 'Start recorder node',
                                     self._launch_recorder, width=22)
        self._launch_btn.pack(side='left', padx=(0, 8))
        self._kill_btn = self._btn(row, 'Stop recorder node',
                                   self._stop_recorder_node, 'danger',
                                   width=22, state='disabled')
        self._kill_btn.pack(side='left')

        self._sliders_btn = self._btn(card, 'Restart joint sliders',
                                      self._restart_joint_sliders, 'ghost',
                                      width=28, state='disabled')
        self._sliders_btn.pack(anchor='w', pady=(4, 0))
        self._btn(card, 'Install desktop shortcut',
                  self._install_shortcut, 'ghost', width=28).pack(anchor='w')

    # ── Record card ───────────────────────────────────────────────────────────

    def _build_record(self):
        card = self._card(self); card.pack(fill='x', pady=(0, 8))

        hdr = tk.Frame(card, bg=BG_S); hdr.pack(fill='x', pady=(0, 8))
        self._hlabel(hdr, 'Recording').pack(side='left')
        self._rec_ind = tk.Label(hdr, text='', bg=BG_S,
                                 fg=C_ERR, font=('Helvetica', 9, 'bold'))
        self._rec_ind.pack(side='right')

        # Posing mode
        row = tk.Frame(card, bg=BG_S); row.pack(fill='x', pady=(0, 6))
        tk.Label(row, text='Posing mode', width=12, anchor='w',
                 bg=BG_S, fg=FG2, font=FL).pack(side='left')
        self._posing_mode_var = tk.StringVar(value='admittance')
        for label, value in (('Admittance', 'admittance'),
                              ('Compliant torque', 'compliant_torque')):
            tk.Radiobutton(row, text=label, variable=self._posing_mode_var,
                           value=value, command=self._on_mode_changed,
                           bg=BG_S, fg=FG2, activebackground=BG_S,
                           selectcolor=BG_I, font=FL,
                           bd=0, highlightthickness=0).pack(side='left', padx=(6, 0))

        # Torque sub-section (shown only in compliant_torque mode)
        self._torque_section = tk.Frame(card, bg=BG_I,
                                        highlightbackground=BDR,
                                        highlightthickness=1,
                                        padx=8, pady=6)
        for attr, label, btn_text, btn_cmd, default, default_fg in [
            ('_bias',  'Torque bias',  'Confirm bias zeroed', self._confirm_bias,
             'Not confirmed', C_WARN),
            ('_fault', 'Safety fault', 'Reset fault',         self._reset_fault,
             'No fault',       FGM),
        ]:
            row = tk.Frame(self._torque_section, bg=BG_I)
            row.pack(fill='x', pady=(0, 4))
            tk.Label(row, text=label, width=12, anchor='w',
                     bg=BG_I, fg=FG2, font=FL).pack(side='left')
            var = tk.StringVar(value=default)
            lbl = tk.Label(row, textvariable=var, bg=BG_I, fg=default_fg, font=FL)
            lbl.pack(side='left', padx=(4, 8))
            self._btn(row, btn_text, btn_cmd, 'ghost', width=20).pack(side='left')
            setattr(self, f'{attr}_var', var)
            setattr(self, f'{attr}_lbl', lbl)

        btn_row = tk.Frame(card, bg=BG_S); btn_row.pack(pady=(6, 0))
        self._start_btn = self._btn(btn_row, 'Start recording',
                                    self._start_recording, width=20, state='disabled')
        self._start_btn.pack(side='left', padx=(0, 8))
        self._stop_btn = self._btn(btn_row, 'Stop recording',
                                   self._stop_recording, 'danger',
                                   width=20, state='disabled')
        self._stop_btn.pack(side='left')

    def _on_mode_changed(self):
        if self._posing_mode_var.get() == 'compliant_torque':
            self._torque_section.pack(fill='x', pady=(0, 6),
                                      before=self._start_btn.master)
        else:
            self._torque_section.pack_forget()

    def _confirm_bias(self):
        self._log('Confirming torque sensor bias zeroed...')
        self._bg_call(self.node.call_confirm_bias, on_ok=lambda: (
            self._bias_var.set('Confirmed'),
            self._bias_lbl.config(fg=C_OK),
        ))

    def _reset_fault(self):
        self._log('Resetting safety fault...')
        self._bg_call(self.node.call_reset_fault, on_ok=lambda: (
            self._fault_var.set('No fault'),
            self._fault_lbl.config(fg=FGM),
        ))

    # ── Replay card ───────────────────────────────────────────────────────────

    def _build_replay(self):
        card = self._card(self); card.pack(fill='x', pady=(0, 8))
        self._hlabel(card, 'Replay').pack(fill='x', pady=(0, 8))

        row = tk.Frame(card, bg=BG_S); row.pack(fill='x', pady=(0, 8))
        tk.Label(row, text='Speed', width=10, anchor='w',
                 bg=BG_S, fg=FG2, font=FL).pack(side='left')
        self._speed_var = tk.DoubleVar(value=1.0)
        self._speed_lbl = tk.Label(row, text='1.0x', width=5,
                                   bg=BG_S, fg=FG, font=FB)
        self._speed_var.trace_add('write', lambda *_: self._speed_lbl.config(
            text=f'{self._speed_var.get():.1f}x'))
        tk.Scale(row, variable=self._speed_var, from_=0.1, to=3.0,
                 resolution=0.1, orient='horizontal', length=200,
                 bg=BG_S, fg=FG, troughcolor=BG_I,
                 highlightthickness=0, showvalue=False, bd=0).pack(side='left', padx=(6, 4))
        self._speed_lbl.pack(side='left')

        row = tk.Frame(card, bg=BG_S); row.pack(fill='x', pady=(0, 4))
        self._replay_btn = self._btn(row, 'Replay last recording',
                                     self._replay_last, width=24, state='disabled')
        self._replay_btn.pack(side='left', padx=(0, 8))
        self._btn(row, 'Browse and replay...',
                  self._browse_replay, 'ghost', width=20).pack(side='left')

    # ── Log card ──────────────────────────────────────────────────────────────

    def _build_log(self):
        card = self._card(self); card.pack(fill='both', expand=True)
        self._hlabel(card, 'Log').pack(fill='x', pady=(0, 6))
        self._logw = scrolledtext.ScrolledText(
            card, height=9, state='disabled',
            bg=BG_I, fg=FG, insertbackground=FG,
            font=FM, relief='flat', wrap='word', padx=6, pady=4)
        self._logw.pack(fill='both', expand=True)
        self._logw.bind('<Button-1>', lambda e: self._logw.focus_set())
        for seq in ('<Control-c>', '<Control-C>', '<Control-Insert>',
                    '<Command-c>', '<Command-C>', '<Button-3>'):
            self._logw.bind(seq, lambda e: (
                self._logw.event_generate('<<Copy>>'), 'break'))

    # ── Status poll ───────────────────────────────────────────────────────────

    def _poll_status(self):
        rec_ok    = self.node.recorder_available()
        proc      = self._procs.get('recorder')
        alive     = proc is not None and proc.poll() is None
        rsp_alive = (p := self._procs.get('rsp'))  and p.poll() is None
        jsb_alive = (p := self._procs.get('jsb'))  and p.poll() is None

        if rec_ok:
            self._set_status('recorder', 'Running', 'success')
            if self._fake_mode and rsp_alive:
                self._set_status('robot',
                                 'RViz running' if jsb_alive else 'Simulated (sliders off)',
                                 'success' if jsb_alive else 'warning')
            elif self._fake_mode:
                self._set_status('robot', 'Simulated', 'warning')
            else:
                self._set_status('robot', 'Connected', 'success')
            self._start_btn.config(state='normal')
            self._stop_btn.config(state='normal')
            self._launch_btn.config(state='disabled')
            self._kill_btn.config(state='normal' if alive else 'disabled')
        elif alive:
            self._set_status('recorder', 'Starting...', 'warning')
            self._set_status('robot', 'Connecting...', 'warning')
            for b in (self._start_btn, self._stop_btn, self._launch_btn):
                b.config(state='disabled')
            self._kill_btn.config(state='normal')
        else:
            self._set_status('recorder', 'Not running', 'danger')
            self._set_status('robot', 'Unknown', 'neutral')
            for b in (self._start_btn, self._stop_btn, self._kill_btn):
                b.config(state='disabled')
            self._launch_btn.config(state='normal')
            self._bias_var.set('Not confirmed')
            self._bias_lbl.config(fg=C_WARN)

        self._sliders_btn.config(
            state='normal' if (self._fake_mode and rsp_alive and not jsb_alive)
            else 'disabled')

        if self._recording:
            self._set_status('state', 'Recording', 'recording')
            self._rec_ind.config(text='RECORDING')
        else:
            self._set_status('state', 'Idle', 'neutral')
            self._rec_ind.config(text='')

        if self._last_file:
            self._replay_btn.config(state='normal')
            self._last_var.set(os.path.basename(self._last_file))

        self.after(1000, self._poll_status)

    # ── Node launch ───────────────────────────────────────────────────────────

    def _launch_recorder(self):
        if 'recorder' in self._procs and self._procs['recorder'].poll() is None:
            self._log('Recorder already running.', 'warning'); return
        self._fake_mode = self._fake_var.get()
        ip = self._ip_var.get().strip() or '192.168.0.10'
        if self._fake_mode:
            self._spawn('recorder',
                        [f'FAKE_HARDWARE=1 ROBOT_IP={ip} ros2 run rec_rep2 recorder'],
                        'success')
        else:
            self._spawn('recorder',
                        [f'ros2 launch rec_rep2 rec_rep2.launch.py robot_ip:={ip}'],
                        'success')
        if self._fake_mode:
            self._spawn('rsp', [
                f'xacro {XACRO_PATH} {XACRO_ARGS} > {URDF_TMP}',
                f'ros2 run robot_state_publisher robot_state_publisher '
                f'--ros-args -p "robot_description:=$(cat {URDF_TMP})"',
            ], 'warning')
            self._spawn('rviz', [f'ros2 run rviz2 rviz2 -d {RVIZ_CFG}'], 'warning')
            self._launch_joint_sliders()

    def _launch_joint_sliders(self):
        self._spawn('jsb',
                    ['ros2 run joint_state_publisher_gui joint_state_publisher_gui'],
                    'warning')
        self._log('Drag sliders to pose the robot.')

    def _restart_joint_sliders(self):
        self._kill('jsb')
        self._launch_joint_sliders()

    def _stop_recorder_node(self):
        for name in ('recorder', 'jsb', 'rviz', 'rsp'):
            self._kill(name)
        self._fake_mode = False

    # ── Shortcut install ──────────────────────────────────────────────────────

    def _install_shortcut(self):
        script_dir = os.path.join(SCRIPTS_DIR, 'scripts')
        os.makedirs(script_dir, exist_ok=True)
        launcher = os.path.join(script_dir, 'launch_gui.sh')
        with open(launcher, 'w') as f:
            f.write('#!/bin/bash\n'
                    'source /opt/ros/humble/setup.bash\n'
                    f'source {WS_OVERLAY}\n'
                    'exec ros2 run rec_rep2 gui\n')
        os.chmod(launcher, 0o755)

        desktop_dir = os.path.expanduser('~/.local/share/applications')
        os.makedirs(desktop_dir, exist_ok=True)
        desktop = os.path.join(desktop_dir, 'rec_rep2_control.desktop')
        with open(desktop, 'w') as f:
            f.write(f'[Desktop Entry]\nVersion=1.0\nType=Application\n'
                    f'Name=rec_rep2 Control Panel\n'
                    f'Comment=Motion Record & Replay for Kinova Gen3\n'
                    f'Exec={launcher}\nIcon=utilities-terminal\n'
                    f'Terminal=false\nCategories=Science;\n')
        subprocess.run(['update-desktop-database', desktop_dir], check=False)
        self._log(f'Desktop shortcut installed: {desktop}', 'success')
        messagebox.showinfo('Shortcut installed',
                            f'Desktop entry:\n{desktop}\nLauncher:\n{launcher}')

    # ── Recording ─────────────────────────────────────────────────────────────

    def _start_recording(self):
        self._log('Calling start_recording...')
        use_torque = self._posing_mode_var.get() == 'compliant_torque'
        def _w():
            ok, msg = self.node.call_set_mode(use_torque)
            if not ok:
                self._log(f'[FAIL] set_posing_mode: {msg}', 'danger'); return
            ok, msg = self.node.call_start()
            self._log(('[OK] ' if ok else '[FAIL] ') + msg,
                      'success' if ok else 'danger')
            if ok: self._recording = True
        threading.Thread(target=_w, daemon=True).start()

    def _stop_recording(self):
        self._log('Calling stop_recording...')
        def _w():
            ok, msg = self.node.call_stop()
            self._log(('[OK] ' if ok else '[FAIL] ') + msg,
                      'success' if ok else 'danger')
            if ok:
                self._recording = False
                parts = msg.split(' to ')
                if len(parts) == 2:
                    self._last_file = parts[1].strip()
        threading.Thread(target=_w, daemon=True).start()

    # ── Replay ────────────────────────────────────────────────────────────────

    def _replay_last(self):
        if not self._last_file:
            messagebox.showwarning('No file', 'No recording made yet.'); return
        self._replay_file(self._last_file)

    def _browse_replay(self):
        path = filedialog.askdirectory(
            title='Select trajectory bag',
            initialdir=BAGS_DIR if os.path.isdir(BAGS_DIR) else os.path.expanduser('~'))
        if path: self._replay_file(path)

    def _replay_file(self, path):
        if not path or not os.path.exists(path):
            messagebox.showerror('Replay error', f'Cannot open: {path}'); return
        speed = self._speed_var.get()
        self._log(f'Replaying {os.path.basename(path)} at {speed:.1f}x...')
        if self._fake_mode:
            self._kill('jsb')
            self._log('Joint sliders paused for replay.', 'warning')
            cmd = self._ros_cmd([
                f'FAKE_HARDWARE=1 ros2 run rec_rep2 fake_replayer {path} {speed:.2f} --storage sqlite3'])
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, preexec_fn=os.setsid)
            def _monitor():
                self._tail(proc, 'fake_replay'); proc.wait()
                if (p := self._procs.get('rsp')) and p.poll() is None:
                    self._log('Replay done.', 'success')
            threading.Thread(target=_monitor, daemon=True).start()
        else:
            cmd = self._ros_cmd([
                f'ros2 run rec_rep2 replayer {path} {speed:.2f} --storage sqlite3'])
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT)
            threading.Thread(target=self._tail, args=(proc, 'replayer'),
                             daemon=True).start()

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def _on_close(self):
        p = self._procs.get('recorder')
        if p and p.poll() is None:
            if messagebox.askyesno('Quit', 'Recorder is running. Stop it before closing?'):
                self._stop_recorder_node()
        else:
            for name in ('jsb', 'rviz', 'rsp'):
                self._kill(name)
        self.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = GuiNode()
    App(node).mainloop()
    rclpy.shutdown()