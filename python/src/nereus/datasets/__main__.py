"""Command line: ``nereus-dataset {generate,plan,render,export,preview,check-classes}``."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from nereus.packs import PackError

from ._documents import load_document
from ._mapping import class_order_problems
from .compare import Options, compare
from .export import FORMATS, export
from .plan import Overrides, describe, plan
from .preview import preview
from .render import render


def _overrides(arguments: argparse.Namespace) -> Overrides:
    return Overrides(
        tasks=arguments.task,
        count=arguments.count,
        range_m=arguments.range_m,
        bearing_deg=arguments.bearing_deg,
        elevation_deg=arguments.elevation_deg,
        altitude_m=arguments.altitude_m,
        resolution=arguments.resolution,
        seed=arguments.seed,
        environments=arguments.environment,
        environment_mode=arguments.environment_mode,
    )


def _plan(dataset: str, out: Path, arguments: argparse.Namespace) -> Path:
    result = plan(Path(dataset), out, _overrides(arguments))
    for warning in result.warnings:
        print(f"WARNING: {warning}", file=sys.stderr)
    total = sum(block["count"] for block in result.job["samples"])
    width, height = result.job["camera"]["resolution_px"]
    print(
        f"planned {result.job['dataset']}: {total} samples, {result.job['camera']['sensor']} "
        f"{width}x{height}, seed {result.job['seed']} -> {result.job_path}"
    )
    for line in describe(result.job):
        print(line)
    return result.job_path.parent


def _formats(text: str) -> list[str]:
    formats = [item.strip() for item in text.split(",") if item.strip()]
    unknown = [item for item in formats if item not in FORMATS]
    if unknown or not formats:
        raise argparse.ArgumentTypeError(f"formats from {', '.join(FORMATS)}")
    return formats


def _export(folder: Path, fmt: str, out: Path, arguments: argparse.Namespace) -> None:
    labels = Path(arguments.labels) if getattr(arguments, "labels", None) else None
    summary = export(folder, fmt, out, labels=labels, model=getattr(arguments, "model", None))
    for line in summary.report():
        print(line)


def _run_generate(arguments: argparse.Namespace) -> int:
    root = Path(arguments.out)
    folder = _plan(arguments.dataset, root / "render", arguments)
    render(folder, workers=arguments.workers, renderer=_path(arguments.renderer))
    if not arguments.no_export:
        for fmt in arguments.formats:
            _export(folder, fmt, root / fmt, arguments)
        sheet = preview(folder, root / "preview.jpg")
        print(f"preview -> {sheet}")
    return 0


def _run_plan(arguments: argparse.Namespace) -> int:
    _plan(arguments.dataset, Path(arguments.out), arguments)
    return 0


def _run_render(arguments: argparse.Namespace) -> int:
    render(Path(arguments.dir), workers=arguments.workers, renderer=_path(arguments.renderer))
    return 0


def _run_export(arguments: argparse.Namespace) -> int:
    folder = Path(arguments.dir)
    out = Path(arguments.out) if arguments.out else folder / arguments.format
    _export(folder, arguments.format, out, arguments)
    return 0


def _run_preview(arguments: argparse.Namespace) -> int:
    folder = Path(arguments.dir)
    out = Path(arguments.out) if arguments.out else folder / "preview.jpg"
    labels = Path(arguments.labels) if arguments.labels else None
    sheet = preview(
        folder,
        out,
        count=arguments.count,
        environments=arguments.environment,
        labels=labels,
        model=arguments.model,
    )
    print(f"preview -> {sheet}")
    return 0


def _run_environments(arguments: argparse.Namespace) -> int:
    options = Options(
        tasks=arguments.task,
        views=arguments.views,
        environments=arguments.environment,
        variants=[item.strip() for item in arguments.variants.split(",") if item.strip()],
        settings=arguments.set or [],
        labels=arguments.labels,
        resolution=arguments.resolution,
        renderer=_path(arguments.renderer),
        workers=arguments.workers,
        force=arguments.force,
    )
    for sheet in compare(Path(arguments.dataset), Path(arguments.out), options):
        print(f"sheet -> {sheet}")
    return 0


def _run_check_classes(arguments: argparse.Namespace) -> int:
    labels = load_document(Path(arguments.labels), "labels")
    problems, notes = class_order_problems(labels.data, Path(arguments.yolo_config))
    for note in notes:
        print(note)
    if problems:
        raise PackError(problems)
    return 0


def _path(value: str | None) -> Path | None:
    return Path(value) if value else None


def _spec_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("dataset", help="dataset folder (dataset.yaml) or file")
    parser.add_argument("--out", required=True, help="output folder")
    parser.add_argument(
        "--task", action="append", help="only this task (repeatable; 'background' for that block)"
    )
    parser.add_argument("--count", type=int, help="samples per selected task")
    parser.add_argument(
        "--range-m", nargs=2, type=float, metavar=("LOW", "HIGH"), help="approach range"
    )
    parser.add_argument("--bearing-deg", type=float, help="approach bearing half-width")
    parser.add_argument(
        "--elevation-deg", nargs=2, type=float, metavar=("LOW", "HIGH"), help="approach elevation"
    )
    parser.add_argument(
        "--altitude-m", nargs=2, type=float, metavar=("LOW", "HIGH"), help="overhead altitude"
    )
    parser.add_argument("--resolution", help="native or WIDTHxHEIGHT")
    parser.add_argument("--seed", type=int, help="dataset seed")
    parser.add_argument(
        "--environment",
        action="append",
        help="only environments with this id (repeatable; globs, e.g. 'murky*')",
    )
    parser.add_argument(
        "--environment-mode", choices=("weighted", "sweep"), help="pick by weight or cycle evenly"
    )


def _label_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--labels", help="label pack (default: the one the plan used)")
    parser.add_argument("--model", help="label-pack model (default: the one the plan used)")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="nereus-dataset")
    commands = parser.add_subparsers(dest="command", required=True)

    generate = commands.add_parser("generate", help="plan, render and export a dataset")
    _spec_options(generate)
    generate.add_argument("--workers", type=int, default=2, help="parallel renderer shards")
    generate.add_argument(
        "--formats", type=_formats, default=["yolo-seg"], help="comma-separated export formats"
    )
    generate.add_argument("--renderer", help="nereus-dataset-render executable")
    generate.add_argument("--no-export", action="store_true", help="render only")
    generate.set_defaults(run=_run_generate)

    plan_ = commands.add_parser("plan", help="write OUT/job.json only")
    _spec_options(plan_)
    plan_.set_defaults(run=_run_plan)

    render_ = commands.add_parser("render", help="render DIR/job.json as parallel shards")
    render_.add_argument("dir", help="planned folder (holds job.json)")
    render_.add_argument("--workers", type=int, default=2, help="parallel renderer shards")
    render_.add_argument("--renderer", help="nereus-dataset-render executable")
    render_.set_defaults(run=_run_render)

    export_ = commands.add_parser("export", help="records -> one YOLO dataset")
    export_.add_argument("dir", help="rendered folder (records/, images/, ids/)")
    export_.add_argument("--format", required=True, choices=FORMATS)
    export_.add_argument("--out", help="dataset folder (default DIR/<format>)")
    _label_options(export_)
    export_.set_defaults(run=_run_export)

    preview_ = commands.add_parser("preview", help="contact sheet with label overlays")
    preview_.add_argument("dir", help="rendered folder")
    preview_.add_argument("--count", type=int, default=16, help="samples on the sheet")
    preview_.add_argument(
        "--environment", action="append", help="only samples of these environments (globs)"
    )
    preview_.add_argument("--out", help="image to write (default DIR/preview.jpg)")
    _label_options(preview_)
    preview_.set_defaults(run=_run_preview)

    grid = commands.add_parser(
        "environments", help="same views under every environment at its min / mid / max"
    )
    grid.add_argument("dataset", help="dataset folder (dataset.yaml) or file")
    grid.add_argument("--out", required=True, help="output folder (sheets, views/, grid/)")
    grid.add_argument("--task", action="append", help="only this task (repeatable)")
    grid.add_argument("--views", type=int, default=3, help="views per task, near to far")
    grid.add_argument("--environment", action="append", help="only these environments (globs)")
    grid.add_argument("--variants", default="min,mid,max", help="range ends to show")
    grid.add_argument(
        "--set",
        action="append",
        metavar="GROUP.KEY=VALUE",
        help="override a value in every environment, e.g. water.scattering_scale=[1.2,1.6]",
    )
    grid.add_argument(
        "--labels", action=argparse.BooleanOptionalAction, default=True, help="label overlays"
    )
    grid.add_argument("--resolution", default="960x600", help="native or WIDTHxHEIGHT")
    grid.add_argument("--renderer", help="nereus-dataset-render executable")
    grid.add_argument("--workers", type=int, default=2, help="parallel renderer shards")
    grid.add_argument("--force", action="store_true", help="reuse an --out made for another spec")
    grid.set_defaults(run=_run_environments)

    check = commands.add_parser(
        "check-classes", help="compare model class order with a YOLO parameter file"
    )
    check.add_argument("labels", help="label pack folder or file")
    check.add_argument(
        "--yolo-config", required=True, help="file with <camera>_class_id_map strings"
    )
    check.set_defaults(run=_run_check_classes)

    arguments = parser.parse_args(argv)
    try:
        result: int = arguments.run(arguments)
    except PackError as error:
        print(f"nereus-dataset {arguments.command}: failed", file=sys.stderr)
        for problem in error.problems:
            print(f"  {problem}", file=sys.stderr)
        return 1
    return result


if __name__ == "__main__":
    sys.exit(main())
