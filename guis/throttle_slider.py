import tkinter as tk
from tkinter import ttk
import threading
import time

class ThrottleUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Throttle Control")
        self.root.geometry("400x300")
        self.root.configure(bg='#2c2c2c')
        
        # Throttle value (0-100)
        self.throttle_value = tk.DoubleVar(value=0)
        
        # Create main frame
        main_frame = tk.Frame(root, bg='#2c2c2c')
        main_frame.pack(expand=True, fill='both', padx=20, pady=20)
        
        # Title
        title_label = tk.Label(
            main_frame, 
            text="THROTTLE CONTROL", 
            font=('Arial', 16, 'bold'),
            fg='white',
            bg='#2c2c2c'
        )
        title_label.pack(pady=(0, 20))
        
        # Throttle percentage display
        self.percentage_label = tk.Label(
            main_frame,
            text="0%",
            font=('Arial', 32, 'bold'),
            fg='#00ff00',
            bg='#2c2c2c'
        )
        self.percentage_label.pack(pady=(0, 20))
        
        # Vertical slider frame
        slider_frame = tk.Frame(main_frame, bg='#2c2c2c')
        slider_frame.pack(expand=True, fill='both')
        
        # Create vertical slider
        self.throttle_slider = tk.Scale(
            slider_frame,
            from_=100,  # Top is 100%
            to=0,       # Bottom is 0%
            orient=tk.VERTICAL,
            variable=self.throttle_value,
            command=self.on_throttle_change,
            length=200,
            width=30,
            font=('Arial', 10),
            bg='#404040',
            fg='white',
            activebackground='#606060',
            highlightbackground='#2c2c2c',
            troughcolor='#1a1a1a',
            sliderlength=30
        )
        self.throttle_slider.pack(side=tk.LEFT, padx=20, expand=True)
        
        # Status indicators frame
        status_frame = tk.Frame(slider_frame, bg='#2c2c2c')
        status_frame.pack(side=tk.RIGHT, fill='y', padx=20)
        
        # Status labels
        self.status_labels = []
        status_levels = [
            ("MAX", 90, '#ff0000'),
            ("HIGH", 70, '#ff8800'),
            ("MED", 50, '#ffff00'),
            ("LOW", 30, '#00ff00'),
            ("IDLE", 10, '#888888')
        ]
        
        for text, threshold, color in status_levels:
            label = tk.Label(
                status_frame,
                text=text,
                font=('Arial', 12, 'bold'),
                fg='#333333',
                bg='#2c2c2c',
                width=6
            )
            label.pack(pady=5)
            self.status_labels.append((label, threshold, color))
        
        # Control buttons frame
        button_frame = tk.Frame(main_frame, bg='#2c2c2c')
        button_frame.pack(fill='x', pady=(20, 0))
        
        # Reset button
        reset_btn = tk.Button(
            button_frame,
            text="RESET",
            command=self.reset_throttle,
            font=('Arial', 12, 'bold'),
            bg='#666666',
            fg='white',
            activebackground='#888888',
            width=10
        )
        reset_btn.pack(side=tk.LEFT, padx=5)
        
        # Full throttle button
        full_btn = tk.Button(
            button_frame,
            text="FULL",
            command=self.full_throttle,
            font=('Arial', 12, 'bold'),
            bg='#cc0000',
            fg='white',
            activebackground='#ff0000',
            width=10
        )
        full_btn.pack(side=tk.RIGHT, padx=5)
        
        # Initialize display
        self.update_display()
        
    def on_throttle_change(self, value):
        """Called when slider value changes"""
        self.update_display()
        
    def update_display(self):
        """Update the percentage display and status indicators"""
        value = int(self.throttle_value.get())
        
        # Update percentage display
        self.percentage_label.config(text=f"{value}%")
        
        # Update percentage color based on value
        if value >= 80:
            color = '#ff0000'  # Red
        elif value >= 60:
            color = '#ff8800'  # Orange
        elif value >= 40:
            color = '#ffff00'  # Yellow
        elif value >= 20:
            color = '#00ff00'  # Green
        else:
            color = '#888888'  # Gray
            
        self.percentage_label.config(fg=color)
        
        # Update status indicators
        for label, threshold, indicator_color in self.status_labels:
            if value >= threshold:
                label.config(fg=indicator_color, bg='#404040')
            else:
                label.config(fg='#333333', bg='#2c2c2c')
    
    def reset_throttle(self):
        """Reset throttle to 0%"""
        self.throttle_value.set(0)
        self.update_display()
    
    def full_throttle(self):
        """Set throttle to 100%"""
        self.throttle_value.set(100)
        self.update_display()
    
    def get_throttle_value(self):
        """Get current throttle value (0-100)"""
        return self.throttle_value.get()

def main():
    # Create main window
    root = tk.Tk()
    
    # Create throttle UI
    throttle_ui = ThrottleUI(root)
    
    # Example of how to use the throttle value in your application
    def monitor_throttle():
        """Example function that monitors throttle changes"""
        while True:
            current_throttle = throttle_ui.get_throttle_value()
            print(f"Current throttle: {current_throttle:.1f}%")
            time.sleep(1)
    
    # Start monitoring in background thread (optional)
    # monitor_thread = threading.Thread(target=monitor_throttle, daemon=True)
    # monitor_thread.start()
    
    # Start the GUI
    root.mainloop()

if __name__ == "__main__":
    main()