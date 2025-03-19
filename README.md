<h1 align="center">🚀 ESP32 Autonomous Maze Solving Robot</h1>

<p align="center">
  <img src="https://github.com/UserAAR/demo-bot/blob/master/build/Gemini_Generated_Image_7hhxb67hhxb67hhx.jpeg" alt="ESP32 Maze Solving Robot" width="700" height="400">
</p>

<p align="center">
  <strong>An AI-powered autonomous robot that can navigate and solve a randomly generated maze using ESP32 and IR sensors.</strong>
</p>

<hr>

<h2>🌟 Project Overview</h2>
<p>
  This project leverages <strong>ESP32 and MicroPython</strong> to develop an autonomous robot capable of solving mazes using <strong>IR sensors and advanced algorithms</strong>.
  The robot detects walls and obstacles using <strong>front and side IR sensors</strong>, enabling it to navigate optimally.
</p>

<ul>
  <li>✅ <strong>Autonomous Maze Solving</strong> – The robot finds its way out without user intervention.</li>
  <li>✅ <strong>Multiple Algorithms Supported</strong> – Right-Hand Rule, DFS, BFS, A*, and Flood-Fill.</li>
  <li>✅ <strong>MicroPython-Based Control</strong> – Lightweight, low-power embedded software.</li>
  <li>✅ <strong>Mapping & Optimization</strong> – The robot can store and analyze its surroundings.</li>
</ul>

<hr>

<h2>⚙️ Technologies & Components</h2>
<table>
  <tr>
    <th>🚀 Component</th>
    <th>🛠 Description</th>
  </tr>
  <tr>
    <td><strong>ESP32</strong></td>
    <td>Microcontroller for processing</td>
  </tr>
  <tr>
    <td><strong>MicroPython</strong></td>
    <td>Embedded Python framework</td>
  </tr>
  <tr>
    <td><strong>IR Sensors</strong></td>
    <td>Detects walls & obstacles</td>
  </tr>
  <tr>
    <td><strong>L298N Motor Driver</strong></td>
    <td>Controls DC motors</td>
  </tr>
  <tr>
    <td><strong>DC Motors</strong></td>
    <td>Provides movement</td>
  </tr>
  <tr>
    <td><strong>Li-Po Battery</strong></td>
    <td>Portable power source</td>
  </tr>
</table>

<hr>

<h2>🧠 Supported Algorithms</h2>
<p>
  This project includes multiple <strong>maze-solving algorithms</strong>, dynamically switchable:
</p>

<ul>
  <li>🔹 <strong>Right-Hand Rule</strong> – Simple but not always optimal.</li>
  <li>🔹 <strong>Left-Hand Rule</strong> – Opposite of right-hand rule.</li>
  <li>🔹 <strong>DFS (Depth-First Search)</strong> – Guarantees a solution but may not find the shortest path.</li>
  <li>🔹 <strong>BFS (Breadth-First Search)</strong> – Ensures shortest path but is computationally expensive.</li>
  <li>🔹 <strong>A* (A-Star) Algorithm</strong> – Finds the most optimal path.</li>
  <li>🔹 <strong>Flood-Fill Algorithm</strong> – Maps the maze and determines the best route.</li>
</ul>

<hr>

<h2>🚀 Installation & Setup</h2>

<h3>1️⃣ Flash MicroPython on ESP32</h3>
<p>If MicroPython is not installed, follow these steps:</p>

<pre>
pip install esptool
esptool.py --port COM3 erase_flash
esptool.py --port COM3 write_flash -z 0x1000 esp32-xxxxxx.bin
</pre>

<h3>2️⃣ Install Required Dependencies</h3>
<pre>
pip install mpremote
</pre>

<h3>3️⃣ Upload & Run the Code</h3>
<pre>
mpremote connect /dev/ttyUSB0
mpremote run main.py
</pre>

<hr>

<h2>📂 Project Structure</h2>
<pre>
/robot_project
│── main.py              # Main control file
│── robot.py             # Robot movement control
│── motor_control.py     # Motor control logic
│── sensors.py           # Sensor data handling
│── algorithms.py        # Maze-solving algorithms
│── strategy.py          # Algorithm selection system
│── factory.py           # Algorithm factory
│── config.py            # Configuration settings
</pre>

<hr>

<h2>📌 Contribution</h2>
<p>Want to contribute? Follow these steps:</p>

<pre>
git clone https://github.com/YUSIFN06/DeepSeek-Maze-Robotex.git
git checkout -b new-feature
git commit -m "Added new feature"
git push origin new-feature
</pre>

<hr>

<p align="center">
  <a href="https://github.com/YUSIFN06/DeepSeek-Maze-Robotex"><img src="https://img.shields.io/github/stars/your-repo?style=social" alt="GitHub Repo"></a>
  <a href="mailto:amil.abdullazada@gmail"><img src="https://img.shields.io/badge/contact-email-blue" alt="Contact Developer"></a>
</p>
