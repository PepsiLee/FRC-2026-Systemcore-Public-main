#!/usr/bin/env python3
"""Read-only structural checks. This does not compile Java or execute robot/tests/simulation."""
from pathlib import Path
import json
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
JAVA = ROOT / 'src/main/java'
errors = []
files = sorted(JAVA.rglob('*.java'))
subsystems = sorted(p.name for p in (JAVA / 'com/team254/frc2025/subsystems').iterdir() if p.is_dir())
if subsystems != ['drive', 'vision']:
    errors.append(f'Unexpected subsystem directories: {subsystems}')
forbidden = re.compile(r'com\.team254\.(?:frc2025\.(?:controlboard|factories|auto|viz|subsystems\.(?:claw|climber|elevator|indexer|intake|led|superstructure|wrist))\b|lib\.(?:subsystems|reefscape)\b)|\b(?:SimulatedRobotState|CommandSimXboxController|ControllerMappings|SimXboxController)\b')
for path in files + sorted((ROOT / 'src/test/java').rglob('*.java')):
    source = path.read_text()
    if forbidden.search(source):
        errors.append(f'Removed dependency referenced: {path.relative_to(ROOT)}')
    for name in re.findall(r'^import\s+(?:static\s+)?(com\.team254\.[\w.*]+);', source, re.M):
        parts = name.split('.')
        # Nested types/static members resolve to the enclosing source file.
        if not any((JAVA.joinpath(*parts[:i]).with_suffix('.java')).is_file()
                   for i in range(1, len(parts) + 1)):
            if not (name.endswith('.*') and JAVA.joinpath(*parts[:-1]).is_dir()):
                errors.append(f'Unresolved local import {name}: {path.relative_to(ROOT)}')

registered = [str(p.relative_to(ROOT)) for p in files if re.search(r'extends\s+SubsystemBase\b|implements\s+Subsystem\b', p.read_text())]
if len(registered) != 2:
    errors.append(f'Expected two subsystem implementations, found {registered}')

for name in ['navgrid.json', 'auto_navgrid.json', 'backoff_navgrid.json']:
    path = ROOT / 'src/main/deploy/pathplanner' / name
    if not path.is_file():
        errors.append(f'Missing pathfinding grid: {name}')

expected_vendors = {
    'AdvantageKit.json': '26.0.2',
    'PathplannerLib.json': '2026.1.2',
    'Phoenix6-frc2026-latest.json': '26.3.0',
    'WPILibNewCommands.json': '1.0.0',
    'maple-sim.json': '0.4.0-beta',
    'photonlib.json': 'v2026.3.4',
}
if {p.name for p in (ROOT / 'vendordeps').glob('*.json')} != set(expected_vendors):
    errors.append('Unexpected or duplicate vendor descriptor files')
for name, version in expected_vendors.items():
    try:
        vendor = json.loads((ROOT / 'vendordeps' / name).read_text())
        if vendor['version'] != version:
            errors.append(f'Unexpected vendor version: {name}')
        if str(vendor.get('frcYear', vendor.get('wpilibYear'))) != '2026':
            errors.append(f'Unexpected vendor WPILib year: {name}')
        for group in ('javaDependencies', 'jniDependencies', 'cppDependencies'):
            for dependency in vendor.get(group, []):
                expected = ('wpilib' if name == 'WPILibNewCommands.json' else
                            '5.0.2' if dependency['groupId'] == 'org.dyn4j' else version)
                if dependency['version'] != expected:
                    errors.append(f'Mixed dependency version: {name}: {dependency}')
    except (KeyError, ValueError, OSError) as error:
        errors.append(f'Invalid vendor descriptor {name}: {error}')
if 'id "edu.wpi.first.GradleRIO" version "2026.2.1"' not in (ROOT / 'build.gradle').read_text():
    errors.append('Unexpected GradleRIO version')
if "frcYear = '2026'" not in (ROOT / 'settings.gradle').read_text():
    errors.append('Unexpected local WPILib Maven year')
waypoint = (JAVA / 'com/team254/lib/pathplanner/path/Waypoint.java').read_text()
if 'import com.pathplanner.lib.util.FlippingUtil;' in waypoint:
    errors.append('Waypoint must use the local 2025 field dimensions')

# The active robot now uses the user's one CANivore/Tuner configuration.
for path in files:
    source = path.read_text()
    if re.search(r'\b(?:PracTunerConstants|SimTunerConstants|kIsPracticeBot)\b', source):
        errors.append(f'Obsolete robot configuration referenced: {path.relative_to(ROOT)}')
    if 'drivebase-climber' in source:
        errors.append(f'Old CAN bus referenced: {path.relative_to(ROOT)}')
drive = (JAVA / 'com/team254/frc2025/subsystems/drive/DriveSubsystem.java').read_text()
if 'kDrivetrain.getModuleLocations()' not in drive or '0.31115' in drive:
    errors.append('PathPlanner module geometry does not follow the Tuner configuration')
sim = (JAVA / 'com/team254/frc2025/subsystems/drive/DriveIOSim.java').read_text()
if 'this.simulationModules = modules.clone()' not in sim:
    errors.append('MapleSim must receive the same modules as the CTRE simulation')
if 'CompTunerConstants.kSimulationLoopPeriod.in(Units.Seconds)' not in sim:
    errors.append('Simulation period does not follow the Tuner configuration')

for path in [ROOT / 'simgui-ds.json', *sorted((ROOT / 'vendordeps').glob('*.json')), *sorted((ROOT / 'layouts').glob('*.json'))]:
    try:
        json.loads(path.read_text())
    except Exception as error:
        errors.append(f'Invalid JSON {path.relative_to(ROOT)}: {error}')

for path in [ROOT / 'README.md', *sorted((ROOT / 'docs/drive-vision').glob('*.md'))]:
    for target in re.findall(r'\]\(([^)]+)\)', path.read_text()):
        if re.match(r'https?://|#', target):
            continue
        target = target.strip('<>').split('#')[0]
        target = re.sub(r':\d+$', '', target)
        if target and not (path.parent / target).resolve().exists():
            errors.append(f'Missing document target in {path.relative_to(ROOT)}: {target}')

diff = subprocess.run(['git', 'diff', '--check'], cwd=ROOT, capture_output=True, text=True)
if diff.returncode:
    errors.append(diff.stdout + diff.stderr)

print(json.dumps({
    'check_type': 'static structure only; no Java compilation or runtime tests',
    'production_java_files': len(files),
    'subsystem_directories': subsystems,
    'subsystem_implementations': registered,
    'vendor_versions': expected_vendors,
    'junit_test_cases_not_executed': sum(len(re.findall(r'@Test\b', p.read_text())) for p in (ROOT / 'src/test/java').rglob('*.java')),
    'errors': errors,
}, ensure_ascii=False, indent=2))
sys.exit(bool(errors))
