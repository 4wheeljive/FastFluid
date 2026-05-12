<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>FastFluid README</title>
<style>
body {
    font-family: Arial, sans-serif;
    line-height: 1.7;
    max-width: 1050px;
    margin: 40px auto;
    padding: 20px;
    background: #ffffff;
    color: #222;
}
h1, h2, h3 {
    color: #111;
    margin-top: 40px;
}
h1 {
    font-size: 42px;
}
h2 {
    border-bottom: 2px solid #eee;
    padding-bottom: 8px;
}
code, pre {
    background: #f4f4f4;
    border-radius: 6px;
    font-family: Consolas, monospace;
}
code {
    padding: 2px 6px;
}
pre {
    padding: 16px;
    overflow-x: auto;
}
table {
    border-collapse: collapse;
    width: 100%;
    margin: 24px 0;
}
th, td {
    border: 1px solid #ccc;
    padding: 12px;
    text-align: left;
}
th {
    background: #f0f0f0;
}
blockquote {
    border-left: 4px solid #999;
    padding-left: 16px;
    color: #555;
    margin-left: 0;
}
</style>
</head>
<body>

<h1>FastFluid</h1>

<p><strong>Realtime fluid simulation for embedded microcontrollers driving LEDs.</strong></p>

<blockquote>
Running Navier–Stokes on hardware that has absolutely no business doing this.
</blockquote>

<p>FastFluid turns LED installations into living dynamic systems instead of playback devices.</p>

<p>Designed for:</p>

<ul>
<li>2D LED matrices</li>
<li>art installations</li>
<li>Burning Man projects</li>
<li>realtime procedural LED visuals</li>
<li>generative LED animations</li>
<li>future integration into FastLED</li>
</ul>

<h2>What Is This?</h2>

<p>FastFluid is an experimental realtime fluid simulation engine for microcontrollers like the ESP32 and Teensy-class hardware, built specifically for LED rendering.</p>

<p>This is not a pre-rendered animation.</p>
<p>This is not noise pretending to be fluid.</p>
<p>This is not a cellular automaton fake-fire effect.</p>

<p><strong>Every frame is generated from a live evolving velocity-field simulation running in realtime on the microcontroller itself.</strong></p>

<p>The simulation includes:</p>

<ul>
<li>advection</li>
<li>diffusion</li>
<li>pressure solving</li>
<li>buoyancy</li>
<li>vorticity confinement</li>
<li>obstacle interaction</li>
<li>realtime force injection</li>
</ul>

<p><strong>The goal is simple:</strong></p>

<p>Make physically-inspired realtime visuals accessible on hardware that has no business running them.</p>

<h2>Why?</h2>

<p>Modern GPUs normalized realtime procedural graphics.</p>

<p>ShaderToy, Winamp MilkDrop, realtime smoke sims, procedural fire, generative visuals — all of that became standard on desktop hardware.</p>

<p>Microcontrollers never really got that evolution.</p>

<p>Most LED effects are still based on:</p>

<ul>
<li>palettes</li>
<li>noise fields</li>
<li>cellular automata</li>
<li>repetitive animations</li>
<li>fake fire</li>
</ul>

<p>FastFluid tries to bring actual realtime fluid dynamics into embedded LED art.</p>

<p>Not scientifically accurate CFD.</p>
<p>Not a physics benchmark.</p>

<p>Just enough physics to create visuals that feel deeply alive.</p>

<h2>Why It Looks Alive</h2>

<p>FastFluid is interesting not because it simulates fluids “accurately”.</p>

<p>It is interesting because systems based on differential equations like Navier–Stokes naturally produce emergent behavior.</p>

<p>Tiny local interactions accumulate into larger evolving structures:</p>

<ul>
<li>vortices</li>
<li>turbulence</li>
<li>flowing motion</li>
<li>self-organizing patterns</li>
<li>chaotic but coherent dynamics</li>
</ul>

<p>The simulation constantly rides the edge between order and chaos.</p>

<p>That is where much of the visual beauty comes from.</p>

<p>You are not animating pixels directly.</p>

<p>You are shaping a dynamic system and watching complex motion emerge from simple mathematical rules in realtime.</p>

<p><strong>That is the real magic behind FastFluid.</strong></p>

<h2>Why This Is Difficult</h2>

<p>Fluid simulation is computationally expensive.</p>

<p>Microcontrollers have:</p>

<ul>
<li>tiny caches</li>
<li>limited RAM</li>
<li>low memory bandwidth</li>
<li>no desktop-class GPU</li>
<li>tight realtime timing constraints for LED output</li>
</ul>

<p>FastFluid exists because the simulation model is aggressively simplified, optimized, and artistically tuned specifically for embedded hardware.</p>

<p>The challenge is not scientific accuracy.</p>

<p><strong>The challenge is achieving visually convincing emergent motion within extremely constrained hardware budgets.</strong></p>

<h2>Current Capabilities</h2>

<ul>
<li>ESP32-S3</li>
<li>ESP32-P4</li>
<li>Teensy 4.x</li>
</ul>

<p>Rendering through:</p>

<ul>
<li>FastLED</li>
<li>WS2812 / NeoPixel</li>
<li>APA102</li>
<li>LED matrices</li>
<li>HUB75 panels</li>
<li>LED strips</li>
<li>basically anything FastLED can drive</li>
</ul>

<p><strong>Current performance:</strong></p>

<ul>
<li>~30 FPS @ 64×48 = 3072 WS2812 LEDs on an ESP32-P4</li>
</ul>

<p>Optimization work is ongoing:</p>

<ul>
<li>SIMD</li>
<li>cache-aware layouts</li>
<li>reduced memory bandwidth</li>
<li>optional fixed-point paths for slower MCUs</li>
</ul>

<h2>Features</h2>

<ul>
<li>Realtime Navier–Stokes-inspired simulation</li>
<li>Smoke simulation</li>
<li>Stylized fluid rendering</li>
<li>Generic velocity fields</li>
<li>Interactive force injection</li>
<li>Artistic rather than scientific tuning</li>
<li>Designed for low-memory embedded systems</li>
<li>Portable architecture</li>
<li>Future FastLED integration planned</li>
</ul>

<h2>Philosophy</h2>

<p>FastFluid is not trying to compete with desktop CFD solvers.</p>

<p>The target is:</p>

<ul>
<li>visually convincing motion</li>
<li>emergent behavior</li>
<li>realtime interaction</li>
<li>low-latency LED rendering</li>
<li>procedural visuals for physical spaces</li>
</ul>

<p>This project lives somewhere between:</p>

<ul>
<li>decorative LED effects</li>
<li>demo-scene coding</li>
<li>generative art</li>
<li>realtime simulation</li>
<li>LED installation culture</li>
</ul>

<h2>Dual-Target Architecture</h2>

<p>FastFluid can currently run on two different ESP32 platforms from a single shared codebase:</p>

<table>
<tr>
<th>Target</th>
<th>Board</th>
<th>LED Driver</th>
<th>BLE</th>
<th>Build</th>
</tr>
<tr>
<td>ESP32-S3</td>
<td>Seeed XIAO ESP32S3</td>
<td>RMT</td>
<td>On-chip</td>
<td>VSCode PlatformIO</td>
</tr>
<tr>
<td>ESP32-P4</td>
<td>ESP32-P4-WIFI6 (Waveshare)</td>
<td>PARLIO</td>
<td>ESP32-C6 via ESP-Hosted VHCI over SDIO</td>
<td>CLI</td>
</tr>
</table>

<h2>How It Works</h2>

<ul>
<li><code>#if __has_include("hosted_ble_bridge.h")</code> → ESP-Hosted BLE init (P4 only)</li>
<li><code>#if defined(CONFIG_IDF_TARGET_ESP32S3)</code> → S3-specific serial config</li>
<li><code>src/board_config.h</code> → pin assignments, matrix dimensions, LED driver selection</li>
</ul>

<h2>Key Files</h2>

<table>
<tr><th>File</th><th>Purpose</th></tr>
<tr><td><code>platformio.ini</code></td><td>ESP32-S3 configuration</td></tr>
<tr><td><code>platformio_p4.ini</code></td><td>ESP32-P4 configuration</td></tr>
<tr><td><code>sdkconfig.defaults</code></td><td>ESP-IDF settings</td></tr>
<tr><td><code>src/boardConfig.h</code></td><td>Hardware abstraction layer</td></tr>
<tr><td><code>src/bleControl.h</code></td><td>NimBLE transport and callbacks</td></tr>
<tr><td><code>src/hosted_ble_bridge.cpp/.h</code></td><td>P4-specific BLE initialization</td></tr>
</table>

<h2>Building</h2>

<h3>ESP32-S3</h3>

<p>Use the standard PlatformIO build/upload buttons inside VSCode.</p>

<h3>ESP32-P4</h3>

<pre>
$env:PLATFORMIO_PROJECT_CONF="platformio_p4.ini"
C:/Users/Jeff/.platformio/penv/Scripts/pio.exe run -c platformio_p4.ini -t upload
</pre>

<h2>Credits</h2>

<p>FastFluid is a collaboration between:</p>

<h3>Stefan Petrick</h3>

<ul>
<li>model design</li>
<li>artistic direction</li>
<li>Python prototyping</li>
<li>distilling fluid simulation models down to the absolute minimum needed for convincing realtime visuals on embedded hardware</li>
</ul>

<h3>Jeff Holman</h3>

<ul>
<li>C++ implementation</li>
<li>optimization</li>
<li>embedded architecture</li>
<li>making the whole thing actually run on microcontrollers</li>
</ul>

<p>Our numeric solver model is heavily inspired by Jos Stam’s paper:</p>

<p><a href="https://pages.cs.wisc.edu/~chaol/data/cs777/stam-stable_fluids.pdf">Stable Fluids (1999)</a></p>

<p>The simulation model is typically explored and tuned first in Python, then ported and heavily optimized for realtime embedded execution.</p>

<h2>Media</h2>

<h3>First realtime fluid experiments</h3>
<p>(put GIF/video here)</p>

<h3>Fire simulation progress</h3>
<p>(put GIF/video here)</p>

<h3>Velocity field visualization</h3>
<p>(put GIF/video here)</p>

</body>
</html>