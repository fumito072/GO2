# Image-generation prompts

These are the final prompts used with the built-in image generation mode.

## 01 — Orbital Cyan

```text
Use case: ui-mockup
Asset type: polished 16:9 desktop web application concept for a quadruped robot operator cockpit
Primary request: Create a high-fidelity, shippable sci-fi redesign of the existing GO2 COCKPIT dashboard. This is a real safety-critical robotics product UI, not cinematic concept art.
Scene/backdrop: full-bleed deep blue-black application canvas, viewed perfectly straight-on, no monitor or device frame.
Style/medium: premium orbital spacecraft command-deck interface; crisp product UI; precise cyan and mint sensor traces; restrained translucent panels; fine technical grid; subtle angular cut corners; minimal controlled glow; highly legible.
Composition/framing: 16:9 landscape. Persistent slim top system rail. Upper perception matrix has three clearly separate panels: a wide front camera view of indoor stairs, a 3D LiDAR point cloud with quadruped marker and heading cone, and a square top-down heightmap with stair edge and vertical elevation legend. Lower area has attitude horizon and pose telemetry, a narrow 12-joint data rail, a manual Q/W/E A/STOP/D S control pad with speed scale, and a continuous AI mission command deck with progress reasoning and log.
Required hierarchy: top rail shows GO2 // COCKPIT, green connected indicator, MOCK, 33.0 V, LOW 18 ms, LiDAR 5 Hz, pose lidar_odom, an obvious DISARMED/ARM toggle, a separate amber STOP control, and a separate red emergency DAMP control. Show native stair task and learned policy M3 controls; DRY RUN selected; LIVE is red and clearly inactive. Show stair metrics height 0.12 m, distance 0.60 m, yaw -0.01, width 0.70 m.
Text (verbatim, use only these short labels where practical): "GO2 // COCKPIT", "接続", "MOCK", "ARM", "STOP", "DAMP", "前面カメラ", "LiDAR", "ハイトマップ", "テレメトリ", "関節", "操縦", "AI任務", "DRY RUN", "LIVE".
Color palette: near-black navy, cyan and mint as normal sensor color; preserve semantic green=safe, amber=warning, red=danger/LIVE, violet=learned policy.
Constraints: straight-on UI screenshot, practical spacing, strong contrast, readable hierarchy, clearly clickable controls, Japanese-friendly typography plus monospace telemetry; safety controls must never look decorative; no company logo; no watermark.
Avoid: fantasy cockpit hardware, physical room, people, cyberpunk city, excessive neon bloom, lens flare, holograms floating outside panels, meaningless dense paragraphs, illegible microtext, random logos, device mockup, tilted perspective.
```

## 02 — Tactical Amber

```text
Use case: ui-mockup
Asset type: polished 16:9 desktop web application concept for a quadruped robot field-operations cockpit
Primary request: Create a high-fidelity tactical sci-fi redesign of the existing GO2 COCKPIT dashboard, optimized for outdoor robot operations and instant safety intervention. It must feel buildable in HTML/CSS.
Scene/backdrop: full-screen matte charcoal and graphite interface, perfectly straight-on, no monitor frame.
Style/medium: rugged near-future field console; aerospace instrumentation; amber-gold primary telemetry lines; etched grid; compact mechanical panel framing; minimal glow; crisp flat surfaces; excellent daylight legibility; not game concept art.
Composition/framing: 16:9 landscape. A fortified top status rail. Three dominant perception bays across the upper half: front camera showing a stair approach with targeting reticle, amber/green LiDAR point cloud with robot outline, and a thermal elevation heightmap with a clear stair edge. Lower half: large attitude/velocity block, 12-joint diagnostics table, tactile-looking keyboard control pad and speed slider, plus an AI mission strip and timestamped event log. Put stair controls and learned-policy controls directly below the heightmap.
Required hierarchy: top rail shows GO2 // COCKPIT, connected, MOCK, battery 33.0 V, sensor freshness, pose source, a guarded ARM switch currently DISARMED, a large amber STOP button, and an isolated red DAMP emergency button. Native climb is green; learned policy M3 is violet; DRY RUN is on; LIVE is locked and red. Show stair metrics height 0.12 m, distance 0.60 m, yaw -0.01, width 0.70 m and a visible DROP/WALL warning channel.
Text (verbatim, use only these short labels where practical): "GO2 // COCKPIT", "CONNECTED", "MOCK", "DISARMED", "ARM", "STOP", "DAMP", "CAMERA", "LiDAR", "HEIGHT MAP", "ATTITUDE", "JOINTS", "CONTROL", "AI MISSION", "DRY RUN", "LIVE".
Color palette: graphite black, warm amber instrumentation, off-white data; preserve green=safe, orange=warning, red=danger/LIVE, violet=learned policy.
Materials/textures: subtle powder-coated panel texture and fine scan-line accents only inside sensor panes.
Constraints: realistic shippable product UI, uncluttered control grouping, readable typography, visible boundaries, no ornamental controls, no company logo, no watermark.
Avoid: steampunk, rusty metal, military insignia, weapon imagery, physical knobs, people, outdoor scene around the UI, excessive grunge, orange monochrome that hides semantic colors, tiny illegible text, random logos, device frame, perspective tilt.
```

## 03 — Aurora Glass

```text
Use case: ui-mockup
Asset type: polished 16:9 desktop web application concept for an advanced robotics laboratory cockpit
Primary request: Create a high-fidelity premium near-future redesign of the existing GO2 COCKPIT dashboard. Preserve the complete operational contract while making the interface feel calm, intelligent, and distinctly science-fiction.
Scene/backdrop: full-screen midnight navy glass interface, perfectly straight-on, no device or room.
Style/medium: refined robotics research-lab product UI; layered dark glass; thin cyan and violet luminous edges; subtle aurora gradients; radial telemetry; generous negative space; sharp typography; practical and shippable, not fantasy concept art.
Composition/framing: 16:9 landscape. Persistent top health rail. A large panoramic front-camera panel anchors the center-left. Two separate stacked sensor panels at right show a cyan/violet 3D LiDAR constellation and a top-down spectral heightmap with stair edge and legend. A slim left rail contains artificial horizon, pose, velocity, body height and command. A narrow far-right rail contains all 12 joint rows. A continuous bottom command deck holds manual movement pad, speed scale, posture actions, AI mission input, reasoning/progress, cancel control and event log. Stair and learned-policy controls bridge the sensor panels and bottom deck.
Required hierarchy: always-visible GO2 // COCKPIT, green connection, MOCK, battery and sensor age, pose lidar_odom, DISARMED/ARM, separate amber STOP and red DAMP. Native climb safe state is green; M3 learned policy is violet; DRY RUN is visibly selected; LIVE is red and inactive. Include height 0.12 m, distance 0.60 m, yaw -0.01, width 0.70 m.
Text (verbatim, use only these short labels where practical): "GO2 // COCKPIT", "接続", "MOCK", "DISARMED", "ARM", "STOP", "DAMP", "CAM", "LiDAR", "HEIGHT", "POSE", "JOINTS", "MANUAL", "AI MISSION", "DRY RUN", "LIVE".
Color palette: midnight navy, ice cyan, aurora violet, soft white; preserve semantic green=safe, amber=warning, red=danger/LIVE, violet=learned policy.
Constraints: practical web UI screenshot, readable at a glance, no critical action hidden in glass effects, distinct panel boundaries, restrained glow, Japanese-capable typography and monospace numeric data, no company logo, no watermark.
Avoid: floating holograms detached from the interface, sci-fi movie bridge, people, robot product photo, decorative planet imagery, magenta overload, excessive blur, unreadable microtext, meaningless paragraphs, random logos, monitor frame, angled perspective.
```


