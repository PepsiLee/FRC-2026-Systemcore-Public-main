// Documentation-only renderer. Does not invoke Java, Gradle, robot tests, or simulation.
// One graph model emits matching Mermaid, Graphviz DOT, SVG, and PNG assets.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const output = path.join(here, 'diagrams');
const deps = process.env.FRC_DOC_NODE_MODULES
    || '/Users/pepsi/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules';
const { instance } = await import(path.join(deps, '@viz-js/viz/dist/viz.js'));
const { default: sharp } = await import(path.join(deps, 'sharp/dist/index.mjs'));
const viz = await instance();
fs.mkdirSync(output, { recursive: true });

const graphs = [
    {
        id: '01-architecture', title: '01 / Drive + Vision architecture', direction: 'TB',
        ranks: [['ps5', 'paths', 'visionio'], ['heading', 'vision'], ['drive', 'state'], ['driveio', 'simstate', 'logs']],
        nodes: [
            ['main', 'Main → Robot\nLifecycle + CommandScheduler', 'host'],
            ['container', 'RobotContainer\nComposition root', 'host'],
            ['ps5', 'CommandPS5Controller(0)\nDirect HID input', 'input'],
            ['heading', 'DriveMaintainingHeadingCommand\nTeleop default command', 'logic'],
            ['tag', 'AprilTagTrackingCommand\nSquare: aim / Triangle: follow 1m', 'logic'],
            ['drive', 'DriveSubsystem\nSwerve output + path controller', 'logic'],
            ['vision', 'VisionSubsystem\nValidate one camera observation', 'logic'],
            ['state', 'RobotState\nPose + motion history', 'state'],
            ['driveio', 'DriveIO\nHardware / Sim', 'io'],
            ['visionio', 'VisionIO\nOne Limelight / one Photon Sim', 'io'],
            ['simstate', 'SimulatedDriveState\nShared Pose2d only', 'state'],
            ['paths', 'com.team11855.lib.pathplanner\nLocal generic planning API', 'logic'],
            ['logs', 'AdvantageKit / DriveViz\nLogs + pose visualization', 'sink'],
        ],
        edges: [
            ['main', 'container', 'constructs'], ['container', 'ps5', 'constructs'],
            ['container', 'drive', 'constructs'], ['container', 'vision', 'constructs'],
            ['ps5', 'heading', 'axes + teleop/connection gate'], ['heading', 'drive', 'requires Drive'],
            ['ps5', 'tag', 'Square / Triangle + gate'], ['vision', 'tag', 'relative tag observation'],
            ['tag', 'drive', 'requires Drive; end stops'],
            ['paths', 'drive', 'trajectory consumer'], ['drive', 'driveio', 'output / input'],
            ['visionio', 'vision', 'camera observations'], ['vision', 'state', 'accepted pose'],
            ['driveio', 'state', 'odometry + motion'], ['state', 'drive', 'control feedback'],
            ['driveio', 'simstate', 'SIM pose'], ['simstate', 'visionio', 'SIM camera scene'],
            ['state', 'logs', 'pose / diagnostics'], ['drive', 'logs', 'module state'],
        ],
    },
    {
        id: '02-control-and-stop', title: '02 / Driver control and stop ownership', direction: 'TB',
        nodes: [
            ['hid', 'PS5 HID / port 0', 'input'],
            ['gate', 'Teleop enabled\nAND HID connected?', 'decision'],
            ['shape', 'Left -Y / -X: signed power 1.5\nRight -X: signed power 2', 'logic'],
            ['turn', 'Turn input above deadband?\nIncludes preserved release settling', 'decision'],
            ['manual', 'FieldCentric\nManual angular rate', 'logic'],
            ['hold', 'FieldCentricFacingAngle\nCapture / hold heading', 'logic'],
            ['options', 'Options rising edge\nKeep XY; blue 0 / red pi\nRefresh heading hold', 'input'],
            ['drive', 'DriveSubsystem\nShared output lock', 'logic'],
            ['path', 'Path controller Notifier\n100 Hz / same output lock', 'logic'],
            ['stop', 'stop()\nClear active trajectory\nSend zero drive request', 'stop'],
            ['exit', 'Command end / mode exit\nDisable / disconnected HID', 'stop'],
            ['io', 'DriveIO → CTRE swerve', 'io'],
            ['auto', 'Autonomous default\nCommands.none()', 'host'],
        ],
        edges: [
            ['hid', 'gate', 'poll each execute'], ['gate', 'shape', 'yes'], ['gate', 'stop', 'no'],
            ['shape', 'turn', 'same speed/PID limits'], ['turn', 'manual', 'turning'],
            ['turn', 'hold', 'released / settled'], ['manual', 'drive', 'request'],
            ['hold', 'drive', 'request'], ['options', 'drive', 'gated pose reset'],
            ['options', 'hold', 'new heading target'], ['path', 'drive', 'trajectory request'],
            ['exit', 'stop', 'stop boundary'], ['stop', 'drive', 'same lock'],
            ['drive', 'io', 'serialized output'], ['auto', 'stop', 'mode entry remains stopped'],
        ],
    },
    {
        id: '03-localization-and-paths', title: '03 / Localization and generic pathfinding', direction: 'TB',
        nodes: [
            ['mount', 'Constants.VisionConstants\nCamera name + mounting position', 'state'],
            ['cameras', 'Single Limelight: limelight-rear\nBlue-origin pose + capture time', 'input'],
            ['sim', 'SimulatedDriveState Pose2d\n→ One Photon camera simulation', 'input'],
            ['visionio', 'VisionIO inputs\nObservations + std deviations', 'io'],
            ['vision', 'VisionSubsystem\nTimestamp / quality checks\nOne accepted camera estimate', 'logic'],
            ['state', 'RobotState\nPose and motion history\nVision estimate callback', 'state'],
            ['ctre', 'DriveIOHardware / CTRE\nVision time conversion\nOdometry telemetry ~250 Hz', 'io'],
            ['api', 'Local AutoBuilder.pathfindToPose\nPose2d + 6 constraint parameters', 'logic'],
            ['grid', '3 retained navgrid JSON files\nteleop / auto / backoff sets', 'input'],
            ['planner', 'LocalADStar\nBackground path planning', 'logic'],
            ['command', 'Local PathfindingCommand\nDrive requirement + trajectory', 'logic'],
            ['controller', 'DriveSubsystem.Controller\n100 Hz feedback + feedforward', 'logic'],
            ['align', 'AutoAlignToPoseCommand\nDirect PID pose alignment', 'logic'],
            ['output', 'DriveSubsystem locked output\n→ DriveIO → Swerve modules', 'sink'],
        ],
        edges: [
            ['mount', 'cameras', 'REAL mounting pose'], ['mount', 'sim', 'SIM axis conversion'],
            ['cameras', 'visionio', 'REAL'], ['sim', 'visionio', 'SIM'],
            ['visionio', 'vision', 'main-loop read'], ['state', 'vision', 'historical pose / motion'],
            ['vision', 'state', 'accepted measurement'], ['state', 'ctre', 'vision callback'],
            ['ctre', 'state', 'odometry / motion'], ['grid', 'planner', 'selected obstacle set'],
            ['api', 'command', 'creates command'], ['command', 'planner', 'planning request'],
            ['planner', 'command', 'new path'], ['command', 'controller', 'trajectory'],
            ['state', 'controller', 'pose feedback'], ['controller', 'output', 'locked output'],
            ['state', 'align', 'pose feedback'], ['align', 'output', 'requires Drive; no path search'],
        ],
    },
    {
        id: '04-apriltag-tracking', title: '04 / AprilTag aim and 1m following', direction: 'TB',
        nodes: [
            ['ps5', 'PS5: Square once / Triangle held\nTriangle wins; Options interrupts', 'input'],
            ['gate', 'Teleop enabled + HID connected\nOptions released', 'decision'],
            ['camera', 'limelight-rear / facing FRONT\ntv + tid + targetpose_cameraspace', 'input'],
            ['time', 'NT timestamp - (cl + tl)\nReject old / malformed data', 'logic'],
            ['mount', 'Camera at robot center\nz = 0.54m, pitch = yaw = 0', 'state'],
            ['geometry', 'AprilTagObservation\nRight/down/forward -> forward/left/up\nCamera-space observation record', 'logic'],
            ['target', 'VisionSubsystem getter\nRelative tag, independent of field pose', 'io'],
            ['command', 'AprilTagTrackingCommand\nLock first valid tag ID\nheading = atan2(y,x), range = hypot(x,y)', 'logic'],
            ['aim', 'Square: rotation only\nWithin 2deg for 0.15s -> finish\n3s timeout', 'logic'],
            ['follow', 'Triangle: hold 1.0m +/- 0.05m\nFar -> forward; near -> reverse\nTurn first if error > 15deg', 'logic'],
            ['output', 'RobotCentric: vx, vy=0, omega\nLimits: 0.6m/s and 1.5rad/s', 'logic'],
            ['drive', 'DriveSubsystem -> DriveIO -> CTRE\nShared lock; clears old path', 'sink'],
            ['stop', 'No valid tag / gate closed / end\nDrive.stop(): zero output\nFollow waits for same tag while held', 'stop'],
        ],
        edges: [
            ['ps5', 'gate', 'input'], ['gate', 'command', 'allowed'], ['gate', 'stop', 'denied'],
            ['camera', 'time', 'one frame'], ['time', 'geometry', 'capture time + position'],
            ['mount', 'command', 'robotToTag transform'], ['geometry', 'target', 'camera-relative observation'],
            ['target', 'command', 'validated age / ID / range'], ['command', 'aim', 'AIM_ONCE'],
            ['command', 'follow', 'FOLLOW_WHILE_HELD'], ['aim', 'output', 'zero translation'],
            ['follow', 'output', 'distance + heading error'], ['output', 'drive', 'setControl'],
            ['command', 'stop', 'invalid / lost / finished'], ['stop', 'drive', 'zero output'],
        ],
    },
];

const colors = {
    host: ['#e6eef8', '#43688f'], input: ['#e5f4f0', '#2d7868'],
    logic: ['#edf2ff', '#5269ad'], io: ['#fff1dd', '#9c7032'],
    state: ['#f2eafb', '#8057a2'], sink: ['#eaf0f4', '#627c91'],
    stop: ['#fce8e8', '#a34b50'], decision: ['#fff7d8', '#9c852d'],
};
const dotString = value => JSON.stringify(value);
const mmdString = value => value.replaceAll('&', '&amp;').replaceAll('"', '&quot;').replaceAll('\n', '<br/>');
for (const graph of graphs) {
    const mmd = [`flowchart ${graph.direction}`];
    for (const [id, label, kind] of graph.nodes) {
        mmd.push(`    ${id}["${mmdString(label)}"]:::${kind}`);
    }
    for (const [from, to, label] of graph.edges) {
        mmd.push(`    ${from} -->|"${mmdString(label)}"| ${to}`);
    }
    for (const [kind, [fill, stroke]] of Object.entries(colors)) {
        mmd.push(`    classDef ${kind} fill:${fill},stroke:${stroke},color:#172d43`);
    }
    fs.writeFileSync(path.join(output, `${graph.id}.mmd`), `${mmd.join('\n')}\n`);
    const dot = [
        'digraph G {',
        `graph [rankdir=${graph.direction}, bgcolor="white", pad=0.35, nodesep=0.38, ranksep=0.65, splines=polyline, fontname="Arial", fontsize=22, fontcolor="#19334c", label=${dotString(graph.title)}, labelloc=t];`,
        'node [shape=box, style="rounded,filled", fontname="Arial", fontsize=14, margin="0.16,0.12", penwidth=1.3, fontcolor="#172d43"];',
        'edge [fontname="Arial", fontsize=11, color="#7a91a5", fontcolor="#435c71", arrowsize=0.7];',
    ];
    for (const [id, label, kind] of graph.nodes) {
        const [fill, stroke] = colors[kind];
        dot.push(`${id} [label=${dotString(label)}, fillcolor="${fill}", color="${stroke}"];`);
    }
    for (const [from, to, label] of graph.edges) {
        dot.push(`${from} -> ${to} [label=${dotString(label)}];`);
    }
    for (const rank of graph.ranks || []) {
        dot.push(`{ rank=same; ${rank.join('; ')}; }`);
    }
    dot.push('}');
    fs.writeFileSync(path.join(output, `${graph.id}.dot`), `${dot.join('\n')}\n`);
    const svg = viz.renderString(dot.join('\n'), { format: 'svg', engine: 'dot' });
    fs.writeFileSync(path.join(output, `${graph.id}.svg`), svg);
    await sharp(Buffer.from(svg), { density: 150 }).png().toFile(path.join(output, `${graph.id}.png`));
    console.log(`${graph.id}: ${graph.nodes.length} nodes / ${graph.edges.length} edges`);
}
