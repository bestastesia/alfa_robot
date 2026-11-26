"""CPU-only independent acceptance for the actual ordered native stage files."""
import argparse,json
from pathlib import Path
from curobo_core.ordered_pick import verify_ordered_result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    check=verify_ordered_result(json.loads(args.result.read_text()))
    args.output.write_text(json.dumps(check,indent=2)+'\n')
    print(json.dumps({k:check[k] for k in ('valid','samples','duration_s','final_updown_m')},ensure_ascii=False))


if __name__=='__main__':main()
