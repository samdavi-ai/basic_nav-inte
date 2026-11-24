"""Optional Jetson/Linux sensors. Unknown readings remain null."""
from pathlib import Path


def _integer(path):
    try:
        return int(path.read_text().strip())
    except (OSError,ValueError):
        return None


class BoardSensors:
    def __init__(self, root=Path("/sys")):
        self.gpu = next((p for p in (root/"class/devfreq").glob("*/device/load")
                         if "gpu" in str(p.resolve()).lower() or "ga10b" in str(p.resolve()).lower()),None)
        self.temps = []
        for path in (root/"class/thermal").glob("thermal_zone*/type"):
            try:
                if path.read_text().strip().lower() in {"cpu-thermal","gpu-thermal","cpu_thermal","gpu_thermal","x86_pkg_temp"}:
                    self.temps.append(path.with_name("temp"))
            except OSError:
                continue
        self.rail = None
        for label in (root/"class/hwmon").glob("hwmon*/in*_label"):
            try:
                if label.read_text().strip().upper() in {"VDD_IN","POM_5V_IN"}:
                    index = label.name.split("_")[0][2:]
                    self.rail = (label.with_name(f"in{index}_input"),label.with_name(f"curr{index}_input"))
                    break
            except OSError:
                continue

    def read(self):
        gpu = _integer(self.gpu) if self.gpu else None
        temps = [v/1000 for p in self.temps if (v:=_integer(p)) is not None and -40000<v<150000]
        rail = [_integer(p) for p in self.rail] if self.rail else []
        return dict(gpu_percent=gpu/10 if gpu is not None and 0<=gpu<=1000 else None,
                    power_w=rail[0]*rail[1]/1e6 if rail and None not in rail else None,
                    temperature_c=max(temps,default=None))
