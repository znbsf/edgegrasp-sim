import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser(description='Compare permission issuance and reception; no motion.')
parser.add_argument('capture', type=Path)
args = parser.parse_args()
rows = [json.loads(line) for line in (args.capture / 'camera/pipeline.jsonl').read_text().splitlines()]
evidence = [(row, json.loads(row['data'])) for row in rows if row['kind'] == 'permission_evidence']
allowed = [(row, item) for row, item in evidence if item['allowed']]
checks = {
    'permission_issuance_observed': bool(allowed),
    'diagnostic_sequence_contiguous': bool(evidence) and all(b[1]['sequence'] == a[1]['sequence'] + 1 for a, b in zip(evidence, evidence[1:])),
    'allowed_decisions_within_source_budget': bool(allowed) and all(0 <= item['decision_ns'] - item['target_source_ns'] <= 200_000_000 for _, item in allowed),
    'allowed_publications_within_source_budget': bool(allowed) and all(item['decision_ns'] <= item['publish_before_ns'] <= item['publish_after_ns'] <= item['target_source_ns'] + 200_000_000 for _, item in allowed),
}
differences = [row['receive_sim_ns'] - item['publish_after_ns'] for row, item in evidence]
report = {'claim': 'publisher_timing_diagnostic_only', 'checks': checks, 'passed': all(checks.values()),
          'permission_rows': len(evidence), 'allowed_rows': len(allowed),
          'receiver_minus_publisher_clock_ns': {'min': min(differences, default=None), 'max': max(differences, default=None)},
          'does_not_reconstruct_prior_unstamped_runs': True}
with (args.capture / 'permission_timing_validation.json').open('x') as stream:
    json.dump(report, stream, indent=2)
print(json.dumps(report))
