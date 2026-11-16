import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description='Reusable UGV A* planner and simulator')
    parser.add_argument('--batch', action='store_true', help='Run all three actual environments without GUI')
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent/'example_outputs')
    parser.add_argument('--radius', type=float, default=None, help='Circular bounding footprint radius in meters; absent means unknown')
    parser.add_argument('--margin', type=float, default=0.)
    parser.add_argument('--connectivity', type=int, choices=[4, 8], default=8)
    parser.add_argument('--verified', action='store_true', help='User confirms measured bounding radius/model; not hardware certification')
    parser.add_argument('--no-inflation', action='store_true')
    parser.add_argument('--clearance-weight', type=float, default=0., help='0 = shortest; GUI defaults to 4')
    parser.add_argument('--extra-clearance', type=float, default=.20, help='Preferred extra clearance beyond hard inflation, meters')
    args = parser.parse_args()
    if args.batch:
        from ugv_planner.batch import run_batch
        from ugv_planner.config import PlannerConfig, Footprint
        cfg = PlannerConfig(args.connectivity, Footprint(args.radius, args.margin, args.verified), not args.no_inflation,
                            clearance_weight=args.clearance_weight, preferred_clearance_m=args.extra_clearance)
        for name, report in run_batch(args.output, cfg).items():
            print(name, report)
    else:
        from ugv_planner.gui import launch
        launch()


if __name__ == '__main__':
    main()
