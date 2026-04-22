# Mission Configuration

## Active Schema

The active mission schema is:

```json
{
  "simulation": {},
  "form": {},
  "environment": {},
  "vehicle": {
    "common": [],
    "input": [],
    "process": [],
    "output": []
  },
  "interaction": {},
  "outputs": {},
  "stop_conditions": []
}
```

The legacy `entities[]` schema is historical and should not be used for new
missions.

## Vehicle Blocks

Use the vehicle blocks as follows:

- `vehicle.common`: static profiles, dataset references, passive asset
  selection
- `vehicle.input`: sensors and measurement-side hardware
- `vehicle.process`: GNC and other command-generation logic
- `vehicle.output`: runtime physical-effect subsystems

If a component changes vehicle physical behavior during the simulation, place
it in `vehicle.output`.

## Example

```json
{
  "simulation": {
    "dt": 0.1,
    "duration": 10.0,
    "integrator": "rk4"
  },
  "form": {
    "components": [
      {
        "type": "form.local_spherical_3dof.point_mass",
        "name": "dynamics",
        "config": {
          "launch_azimuth_rad": 1.5707963267948966,
          "initial_state": {
            "longitude_rad": 1.9198621771937625,
            "latitude_rad": 0.5235987755982988,
            "altitude_m": 60000.0,
            "speed_mps": 3200.0,
            "flight_path_angle_rad": -0.10,
            "heading_angle_rad": -1.5707963267948966
          }
        }
      }
    ]
  },
  "environment": {
    "components": [
      { "type": "environment.spherical_earth", "name": "earth", "config": {} },
      { "type": "environment.standard_atmosphere", "name": "atmosphere", "config": {} },
      { "type": "environment.spherical_gravity", "name": "gravity", "config": {} }
    ]
  },
  "vehicle": {
    "common": [],
    "input": [],
    "process": [
      {
        "type": "vehicle.process.programmed_aoa",
        "name": "guidance",
        "config": {
          "bank_angle_deg": 0.0,
          "schedule_altitude_m": [60000, 45000, 30000, 15000],
          "schedule_angle_of_attack_deg": [20, 12, 10, 8]
        }
      }
    ],
    "output": [
      {
        "type": "mass.constant",
        "name": "mass",
        "config": {
          "asset_file": "framework/data/vehicles/cavh/output/mass_atmospheric_reference.json"
        }
      },
      {
        "type": "aero.table2d",
        "name": "aero",
        "config": {
          "asset_file": "framework/data/vehicles/cavh/output/aero_table2d.json"
        }
      }
    ]
  },
  "interaction": {
    "components": [
      {
        "type": "interaction.local_spherical_3dof.aero_propulsive",
        "name": "interaction",
        "config": {}
      }
    ]
  },
  "outputs": {
    "enabled": false
  }
}
```

## Asset Files

Runtime output components may load assets using `asset_file`.

Canonical runtime asset formats should be text-friendly and reviewable, such
as:

- JSON
- CSV

MATLAB files may still be a generation source, but they should be converted
into the framework runtime asset formats before simulation-time loading.
