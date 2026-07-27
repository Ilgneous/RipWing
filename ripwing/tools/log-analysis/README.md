# Log Analysis & ML Training

Python tooling for parsing flight logs and training the anomaly
detection model.

The log record schema must match `src/telemetry/logger.rs`. Treat that
file as the source of truth and mirror it here.

## Setup
    python -m venv .venv
    source .venv/bin/activate
    pip install numpy pandas scikit-learn matplotlib
