"""Check the read-only cleanup inventory's protection and accounting behavior."""

import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest


SHELL = shutil.which('pwsh') or shutil.which('powershell')
pytestmark = pytest.mark.skipif(os.name != 'nt' or not SHELL, reason='Windows PowerShell inventory')
SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'inspect_blender_cleanup.ps1'


def invoke(plan_path):
    return subprocess.run([SHELL, '-NoProfile', '-File', str(SCRIPT), '-Plan', str(plan_path)],
                          capture_output=True, text=True, encoding='utf-8', errors='replace', timeout=30)


def test_inventory_preserves_files_and_excludes_protected_and_raw(tmp_path):
    protected = tmp_path / 'final'
    protected.mkdir()
    final = protected / 'replay.blend'
    final.write_bytes(b'final')
    archive = tmp_path / 'archive.zip'
    archive.write_bytes(b'abcd')
    raw = tmp_path / 'experiment'
    raw.mkdir()
    recording = raw / 'record.mcap'
    recording.write_bytes(b'raw evidence')
    plan = tmp_path / 'plan.json'
    plan.write_text(json.dumps({'schema_version': 1, 'protected': [str(protected)],
                               'candidates': [
                                   {'path': str(archive), 'kind': 'download_archive', 'reason': 'test'},
                                   {'path': str(protected), 'kind': 'intermediate_replay', 'reason': 'test'},
                                   {'path': str(raw), 'kind': 'intermediate_replay', 'reason': 'test'},
                               ]}), encoding='utf-8')
    result = invoke(plan)
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert report['candidate_bytes'] == 4
    assert report['removed_bytes'] == 0
    assert not report['deletion_authorized_by_report']
    assert report['candidates'][1]['blockers'] == ['protected_path_overlap']
    assert report['candidates'][2]['blockers'] == ['raw_recording_present']
    assert archive.read_bytes() == b'abcd'
    assert final.read_bytes() == b'final'
    assert recording.read_bytes() == b'raw evidence'


def test_overlapping_candidates_rejected_without_deletion(tmp_path):
    candidate = tmp_path / 'intermediate'
    candidate.mkdir()
    archive = candidate / 'archive.zip'
    archive.write_bytes(b'keep')
    plan = tmp_path / 'plan.json'
    plan.write_text(json.dumps({'schema_version': 1, 'protected': [str(tmp_path / 'final')],
                               'candidates': [
                                   {'path': str(candidate), 'kind': 'intermediate_replay', 'reason': 'test'},
                                   {'path': str(archive), 'kind': 'download_archive', 'reason': 'test'},
                               ]}), encoding='utf-8')
    assert invoke(plan).returncode != 0
    assert archive.read_bytes() == b'keep'
