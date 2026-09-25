import csv
import os
from datetime import datetime

class DataLogger:
    def __init__(self):
        self.is_recording = False
        self.recorded_data = []
        self.start_time = 0.0
        self.headers = []

    def start_recording(self, current_time):
        """Starts recording telemetry data."""
        self.is_recording = True
        self.recorded_data.clear()
        self.start_time = current_time
        self.headers = [] # Reset headers for a new recording

    def stop_and_save(self, system_name="system") -> str:
        """
        Stops recording and saves collected telemetry data to a CSV file.
        Returns the file path or None if no data was collected.
        """
        self.is_recording = False
        if not self.recorded_data or not self.headers:
            return None

        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        logs_dir = os.path.join(base_dir, "logs")
        os.makedirs(logs_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(logs_dir, f"{system_name.lower()}_log_{timestamp}.csv")

        try:
            with open(filename, mode="w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(self.headers)
                writer.writerows(self.recorded_data)
            print(f"[DataLogger] Data successfully saved to: {filename}")
            return filename
        except Exception as e:
            print(f"[DataLogger] Error saving CSV: {e}")
            return None

    def _extract_numeric_val(self, item):
        """Helper function extracting the latest numeric value from a dict/list/tuple."""
        if isinstance(item, dict):
            buf = item.get("data", [])
            item = buf[-1] if len(buf) > 0 else 0.0

        if isinstance(item, (list, tuple)):
            item = item[-1] if len(item) > 0 else 0.0

        try:
            return float(item)
        except (ValueError, TypeError):
            return None

    def _is_valid_chart_key(self, key: str) -> bool:
        """Checks if a given key corresponds to valid chart telemetry rather than UI metadata."""
        ignored_keywords = ["status", "color", "title", "text"]
        key_lower = str(key).lower()
        return not any(kw in key_lower for kw in ignored_keywords)

    def sample(self, current_time, system, errors=None):
        """
        Samples a single state point from the system and optional errors if recording is active.
        :param current_time: Current simulation time
        :param system: System instance containing get_charts_data()
        :param errors: Optional dict or list of errors (e.g. {"error_x": 0.05, "error_y": -0.01})
        """
        if not self.is_recording:
            return

        elapsed_time = current_time - self.start_time
        row = [f"{elapsed_time:.4f}"]

        # 1. Fetching data from the system (charts)
        charts_data = system.get_charts_data() if hasattr(system, "get_charts_data") else {}

        # Initialize headers on the first sample
        if not self.headers:
            self.headers = ["Time [s]"]
            
            # Headers from the system
            for chart_key, series_list in charts_data.items():
                if not self._is_valid_chart_key(chart_key):
                    continue

                if isinstance(series_list, list):
                    for idx, series in enumerate(series_list):
                        val = self._extract_numeric_val(series)
                        if val is not None:
                            self.headers.append(f"{chart_key}_series_{idx + 1}")
                else:
                    val = self._extract_numeric_val(series_list)
                    if val is not None:
                        self.headers.append(f"{chart_key}")

            # Headers from explicitly provided errors
            if errors:
                if isinstance(errors, dict):
                    for err_key in errors.keys():
                        self.headers.append(f"err_{err_key}")
                elif isinstance(errors, (list, tuple)):
                    for idx in range(len(errors)):
                        self.headers.append(f"error_{idx + 1}")

        # Populating row with system data
        for chart_key, series_list in charts_data.items():
            if not self._is_valid_chart_key(chart_key):
                continue

            if isinstance(series_list, list):
                for series in series_list:
                    val = self._extract_numeric_val(series)
                    if val is not None:
                        row.append(f"{val:.4f}")
            else:
                val = self._extract_numeric_val(series_list)
                if val is not None:
                    row.append(f"{val:.4f}")

        # Populating row with error data
        if errors:
            if isinstance(errors, dict):
                for err_val in errors.values():
                    num_val = self._extract_numeric_val(err_val)
                    row.append(f"{num_val:.4f}" if num_val is not None else "0.0000")
            elif isinstance(errors, (list, tuple)):
                for err_val in errors:
                    num_val = self._extract_numeric_val(err_val)
                    row.append(f"{num_val:.4f}" if num_val is not None else "0.0000")

        self.recorded_data.append(row)