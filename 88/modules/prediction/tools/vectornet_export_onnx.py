#!/usr/bin/env python3

###############################################################################
# Copyright 2026 The Century Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

"""
Export Apollo VectorNet model (.pt) to ONNX format for TensorRT conversion.

Usage:
  # Step 1: Export to ONNX
  python3 vectornet_export_onnx.py \
      --model_path /century/modules/prediction/data/vectornet_vehicle_model.pt \
      --output_path /tmp/vectornet_vehicle.onnx

  # Step 2: Convert to TensorRT engine (run on target GPU)
  trtexec --onnx=/tmp/vectornet_vehicle.onnx \
          --saveEngine=/century/vectornet_vehicle.engine \
          --fp16

Supports two model formats:
  1. TorchScript (.pt) - loaded with torch.jit.load(), exported directly
  2. State dict only (.pt/.pth) - requires model class definition to reconstruct

The script will auto-detect which format the file uses.
"""

import argparse
import os
import struct
import sys

import torch
import torch.onnx


def inspect_pt_file(model_path):
    """Inspect a .pt file to determine its format and contents."""
    print("=" * 60)
    print("Inspecting: {}".format(model_path))
    print("File size: {:.2f} MB".format(os.path.getsize(model_path) / 1024 / 1024))
    print("=" * 60)

    # Try loading as TorchScript first
    try:
        model = torch.jit.load(model_path, map_location="cpu")
        print("[OK] Successfully loaded as TorchScript model")
        print("")

        # Print model graph
        print("--- Model Graph (first 50 lines) ---")
        graph_str = str(model.graph)
        lines = graph_str.split("\n")
        for line in lines[:50]:
            print("  " + line)
        if len(lines) > 50:
            print("  ... ({} more lines)".format(len(lines) - 50))
        print("")

        # Print parameters
        params = dict(model.named_parameters())
        buffers = dict(model.named_buffers())
        print("--- Parameters ({}) ---".format(len(params)))
        total_params = 0
        for name, param in sorted(params.items()):
            print("  {} : {} ({})".format(name, list(param.shape), param.dtype))
            total_params += param.numel()
        print("  Total: {:,} parameters".format(total_params))
        print("")

        if buffers:
            print("--- Buffers ({}) ---".format(len(buffers)))
            for name, buf in sorted(buffers.items()):
                print("  {} : {} ({})".format(name, list(buf.shape), buf.dtype))
            print("")

        # Try to get forward signature
        print("--- Forward Method ---")
        try:
            forward_schema = model.forward.schema
            print("  Schema: {}".format(forward_schema))
        except Exception:
            print("  (schema not available)")
        print("")

        return "torchscript", model

    except Exception as e:
        print("[INFO] Not a TorchScript model: {}".format(e))
        print("")

    # Try loading as state_dict / checkpoint
    try:
        data = torch.load(model_path, map_location="cpu")

        if isinstance(data, dict):
            if "state_dict" in data:
                state_dict = data["state_dict"]
                print("[OK] Loaded as checkpoint with 'state_dict' key")
                extra_keys = [k for k in data.keys() if k != "state_dict"]
                if extra_keys:
                    print("  Extra keys: {}".format(extra_keys))
            elif "model_state_dict" in data:
                state_dict = data["model_state_dict"]
                print("[OK] Loaded as checkpoint with 'model_state_dict' key")
            else:
                # Check if it looks like a raw state_dict
                has_tensors = any(isinstance(v, torch.Tensor) for v in data.values())
                if has_tensors:
                    state_dict = data
                    print("[OK] Loaded as raw state_dict")
                else:
                    print("[WARN] Dict loaded but no recognizable structure")
                    print("  Keys: {}".format(list(data.keys())))
                    return "unknown", data
        else:
            print("[WARN] Loaded object type: {}".format(type(data)))
            return "unknown", data

        print("")
        print("--- State Dict ({} entries) ---".format(len(state_dict)))
        total_params = 0
        for name, param in sorted(state_dict.items()):
            if isinstance(param, torch.Tensor):
                print("  {} : {} ({})".format(name, list(param.shape), param.dtype))
                total_params += param.numel()
            else:
                print("  {} : {}".format(name, type(param)))
        print("  Total: {:,} parameters".format(total_params))
        print("")

        return "state_dict", state_dict

    except Exception as e:
        print("[ERROR] Failed to load: {}".format(e))
        return "error", None


def export_torchscript_to_onnx(model, output_path, opset_version=13):
    """Export a TorchScript model to ONNX."""
    model.eval()

    # Apollo VectorNet input format (from vectornet_evaluator.cc LoadModel):
    #   target_obstacle_pos:      [1, 20, 2]
    #   target_obstacle_pos_step: [1, 20, 2]
    #   vector_data:              [1, 450, 50, 9]
    #   vector_mask:              [1, 450, 50] bool
    #   polyline_mask:            [1, 450] bool
    #   rand_mask:                [1, 450] bool
    #   polyline_id:              [1, 450, 2]
    #
    # Packed as a tuple input to model.forward()

    target_obs_pos = torch.randn(1, 20, 2)
    target_obs_pos_step = torch.randn(1, 20, 2)
    vector_data = torch.randn(1, 450, 50, 9)
    vector_mask = (torch.randn(1, 450, 50) > 0.9)
    polyline_mask = (torch.randn(1, 450) > 0.9)
    rand_mask = torch.zeros(1, 450).bool()
    polyline_id = torch.randn(1, 450, 2)

    # The model expects a tuple input
    dummy_input = (
        target_obs_pos,
        target_obs_pos_step,
        vector_data,
        vector_mask,
        polyline_mask,
        rand_mask,
        polyline_id,
    )

    print("Exporting to ONNX (opset={})...".format(opset_version))
    print("Input shapes:")
    names = [
        "target_obs_pos", "target_obs_pos_step", "vector_data",
        "vector_mask", "polyline_mask", "rand_mask", "polyline_id"
    ]
    for name, tensor in zip(names, dummy_input):
        print("  {} : {} ({})".format(name, list(tensor.shape), tensor.dtype))

    try:
        # Force legacy (TorchScript-based) ONNX exporter.
        # PyTorch 2.5+ defaults to dynamo-based exporter which cannot
        # handle TorchScript models (RecursiveScriptModule).
        export_kwargs = dict(
            input_names=names,
            output_names=["trajectory"],
            opset_version=opset_version,
            do_constant_folding=True,
        )

        # PyTorch >= 2.5 accepts dynamo=False to select legacy path
        torch_version = tuple(int(x) for x in torch.__version__.split("+")[0].split(".")[:2])
        if torch_version >= (2, 5):
            export_kwargs["dynamo"] = False

        torch.onnx.export(
            model,
            (dummy_input,),  # model.forward expects a tuple wrapped in args
            output_path,
            **export_kwargs,
        )
        print("")
        print("[OK] ONNX export successful: {}".format(output_path))
        print("File size: {:.2f} MB".format(
            os.path.getsize(output_path) / 1024 / 1024))
        return True

    except Exception as e:
        print("")
        print("[ERROR] ONNX export failed: {}".format(e))
        print("")
        print("Common causes:")
        print("  1. Model uses dynamic control flow (if/for with data-dependent conditions)")
        print("  2. Model uses unsupported ops (scatter, gather with complex indexing)")
        print("  3. Input format mismatch (tuple packing)")
        print("")
        print("Workarounds:")
        print("  1. Try torch.onnx.export with a different opset version (11, 12, 14, 16)")
        print("  2. Simplify the model's forward() to avoid unsupported ops")
        print("  3. Use torch.onnx.export(..., operator_export_type=torch.onnx.OperatorExportTypes.ONNX_ATEN_FALLBACK)")
        print("  4. Register custom symbolic functions for unsupported ops")
        return False


def verify_onnx(onnx_path):
    """Verify the exported ONNX model."""
    try:
        import onnx
        model = onnx.load(onnx_path)
        onnx.checker.check_model(model)
        print("[OK] ONNX model validation passed")

        print("  Inputs:")
        for inp in model.graph.input:
            shape = [d.dim_value if d.dim_value > 0 else "?"
                     for d in inp.type.tensor_type.shape.dim]
            print("    {} : {}".format(inp.name, shape))

        print("  Outputs:")
        for out in model.graph.output:
            shape = [d.dim_value if d.dim_value > 0 else "?"
                     for d in out.type.tensor_type.shape.dim]
            print("    {} : {}".format(out.name, shape))

    except ImportError:
        print("[SKIP] onnx package not installed, skipping validation")
    except Exception as e:
        print("[WARN] ONNX validation issue: {}".format(e))


def main():
    parser = argparse.ArgumentParser(
        description="Export Apollo VectorNet model to ONNX")
    parser.add_argument("--model_path", required=True,
                        help="Path to PyTorch model (.pt)")
    parser.add_argument("--output_path", default="",
                        help="Path for output .onnx file (default: same dir as input)")
    parser.add_argument("--opset", type=int, default=13,
                        help="ONNX opset version (default: 13)")
    parser.add_argument("--inspect_only", action="store_true",
                        help="Only inspect the model, do not export")
    args = parser.parse_args()

    if not args.output_path:
        base = os.path.splitext(args.model_path)[0]
        args.output_path = base + ".onnx"

    # Step 1: Inspect the model
    fmt, data = inspect_pt_file(args.model_path)

    if args.inspect_only:
        print("Inspect-only mode, exiting.")
        return

    # Step 2: Export based on format
    if fmt == "torchscript":
        success = export_torchscript_to_onnx(data, args.output_path, args.opset)
        if success:
            verify_onnx(args.output_path)
            print("")
            print("Next step - convert to TensorRT engine:")
            print("  trtexec --onnx={} \\".format(args.output_path))
            print("          --saveEngine={} \\".format(
                args.output_path.replace(".onnx", ".engine")))
            print("          --fp16")
    elif fmt == "state_dict":
        print("[ERROR] This .pt file only contains weights (state_dict),")
        print("        not a complete TorchScript model.")
        print("")
        print("To export to ONNX, you need the model class definition.")
        print("Options:")
        print("  1. Find the Python model code and re-export with torch.jit.script()")
        print("  2. Ask the model author for a TorchScript-traced version")
        print("  3. Reconstruct the model class, load weights, then export")
        sys.exit(1)
    else:
        print("[ERROR] Unrecognized model format.")
        sys.exit(1)


if __name__ == "__main__":
    main()
