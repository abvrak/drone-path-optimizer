import math
import os
import arcpy
import requests
from arcpy.sa import CostDistance, CostPathAsPolyline
from arcpy.sa import Con, IsNull, Raster, Reclassify, RemapRange, Slope, Aspect, Abs, Mod

def parse_xy(value):
    parts = [p.strip() for p in value.split(",")]
    return float(parts[0]), float(parts[1])


def bearing_deg(start_xy, end_xy):
    dx = end_xy[0] - start_xy[0]
    dy = end_xy[1] - start_xy[1]
    angle = math.degrees(math.atan2(dx, dy))
    return (angle + 360.0) % 360.0


def wind_factor(wind_speed, wind_deg, route_bearing):
    angle_diff = abs((wind_deg - route_bearing + 180.0) % 360.0 - 180.0)
    return max(0.6, 1.0 + (wind_speed / 15.0) * (angle_diff / 180.0))


def get_lublin_weather(api_key):
    if not api_key or requests is None:
        return 0.0, 0.0

    url = (
        "https://api.openweathermap.org/data/2.5/weather"
        f"?q=Lublin,PL&appid={api_key}"
    )
    data = requests.get(url, timeout=10).json()
    wind_speed = float(data.get("wind", {}).get("speed", 0.0))
    wind_deg = float(data.get("wind", {}).get("deg", 0.0))
    return wind_speed, wind_deg


def create_point_fc(output_gdb, nmt_raster, name, point_xy):
    fc_path = os.path.join(output_gdb, name)
    if arcpy.Exists(fc_path):
        arcpy.management.Delete(fc_path)

    spatial_ref = arcpy.Describe(nmt_raster).spatialReference
    arcpy.management.CreateFeatureclass(
        output_gdb, name, "POINT", spatial_reference=spatial_ref
    )
    arcpy.management.AddField(fc_path, "Name", "TEXT")
    with arcpy.da.InsertCursor(fc_path, ["SHAPE@", "Name"]) as cursor:
        point = arcpy.Point(point_xy[0], point_xy[1])
        cursor.insertRow([arcpy.PointGeometry(point, spatial_ref), name])
    return fc_path


def build_cost_raster(
    nmt_raster,
    buildings_fc,
    output_gdb,
    wind_speed,
    wind_deg,
    penalty,
    vegetation_raster,
    vegetation_penalty=3.0,
):
    arcpy.CheckOutExtension("Spatial")

    nmt = Raster(nmt_raster)
    arcpy.env.snapRaster = nmt
    arcpy.env.cellSize = nmt.meanCellWidth

    slope = Slope(nmt, "DEGREE")
    aspect = Aspect(nmt)
    slope_cost = Reclassify(
        slope,
        "VALUE",
        RemapRange([[0, 5, 1], [5, 15, 2], [15, 30, 4], [30, 90, 8]]),
    )

    buildings_buffer = os.path.join(output_gdb, "buildings_buffer_10m")
    if arcpy.Exists(buildings_buffer):
        arcpy.management.Delete(buildings_buffer)

    arcpy.analysis.Buffer(
        buildings_fc,
        buildings_buffer,
        "10 Meters",
        dissolve_option="ALL",
    )

    buildings_raster = os.path.join(output_gdb, "buildings_r")
    if arcpy.Exists(buildings_raster):
        arcpy.management.Delete(buildings_raster)

    oid_field = arcpy.Describe(buildings_buffer).OIDFieldName
    arcpy.conversion.PolygonToRaster(
        buildings_buffer, oid_field, buildings_raster, cellsize=nmt.meanCellWidth
    )

    if wind_speed > 0:
        angle_diff = Abs(Mod(aspect - float(wind_deg) + 180.0, 360.0) - 180.0)
        wind_factor_raster = 1.0 + (float(wind_speed) / 15.0) * (angle_diff / 180.0)
        wind_factor_raster = Con(wind_factor_raster < 0.6, 0.6, wind_factor_raster)
        base_cost = slope_cost * wind_factor_raster
    else:
        base_cost = slope_cost
    vegetation = Raster(vegetation_raster)
    vegetation_height = Con(IsNull(vegetation), 0, vegetation - nmt)
    vegetation_height = Con(vegetation_height < 0, 0, vegetation_height)
    vegetation_multiplier = 1.0 + (vegetation_height * float(vegetation_penalty))
    base_cost = base_cost * vegetation_multiplier

    cost_raster = Con(
        IsNull(buildings_raster),
        base_cost,
        base_cost * float(penalty),
    )

    cost_output = os.path.join(output_gdb, "cost_surface")
    cost_raster.save(cost_output)
    return cost_output


def create_3d_path(nmt_raster, path_2d, output_gdb, altitude_offset=0.0):
    output_3d = os.path.join(output_gdb, "drone_path_3d")
    if arcpy.Exists(output_3d):
        arcpy.management.Delete(output_3d)

    prev_snap = arcpy.env.snapRaster
    prev_cell = arcpy.env.cellSize
    prev_extent = arcpy.env.extent
    try:
        arcpy.CheckOutExtension("3D")
        arcpy.CheckOutExtension("Spatial")
        nmt = Raster(nmt_raster)
        with arcpy.EnvManager(
            snapRaster=nmt_raster,
            cellSize=nmt.meanCellWidth,
            extent=nmt.extent,
        ):
            if altitude_offset:
                temp_surface = os.path.join(output_gdb, "nmt_offset")
                if arcpy.Exists(temp_surface):
                    arcpy.management.Delete(temp_surface)
                (nmt + float(altitude_offset)).save(temp_surface)
                arcpy.ddd.InterpolateShape(temp_surface, path_2d, output_3d)
                if arcpy.Exists(temp_surface):
                    arcpy.management.Delete(temp_surface)
            else:
                arcpy.ddd.InterpolateShape(nmt_raster, path_2d, output_3d)
        return output_3d
    except Exception as exc:
        arcpy.AddWarning(f"Nie udało się utworzyć trasy 3D: {exc}")
        return None
    finally:
        arcpy.env.snapRaster = prev_snap
        arcpy.env.cellSize = prev_cell
        arcpy.env.extent = prev_extent


def compute_path(
    workspace,
    nmt_raster,
    buildings_fc,
    output_gdb,
    start_xy,
    end_xy,
    api_key,
    penalty=1000,
    altitude_offset=30.0,
    vegetation_raster=None,
    vegetation_penalty=3.0,
    ):
    arcpy.env.workspace = workspace
    arcpy.env.overwriteOutput = True

    wind_speed, wind_deg = get_lublin_weather(api_key)

    cost_surface = build_cost_raster(
        nmt_raster,
        buildings_fc,
        output_gdb,
        wind_speed,
        wind_deg,
        penalty,
        vegetation_raster,
        vegetation_penalty,
    )
    start_fc = create_point_fc(output_gdb, nmt_raster, "start_pt", start_xy)
    end_fc = create_point_fc(output_gdb, nmt_raster, "end_pt", end_xy)

    arcpy.CheckOutExtension("Spatial")

    cost_dist = os.path.join(output_gdb, "cost_distance")
    back_link = os.path.join(output_gdb, "back_link")
    cost_distance = CostDistance(start_fc, cost_surface, out_backlink_raster=back_link)
    cost_distance.save(cost_dist)

    output_path = os.path.join(output_gdb, "drone_path")
    if arcpy.Exists(output_path):
        arcpy.management.Delete(output_path)

    CostPathAsPolyline(end_fc, cost_dist, back_link, output_path)
    output_3d = create_3d_path(nmt_raster, output_path, output_gdb, altitude_offset)
    return output_3d or output_path


def main():
    workspace = arcpy.GetParameterAsText(0)
    nmt_raster = arcpy.GetParameterAsText(1)
    nmpt_raster = arcpy.GetParameterAsText(2)
    buildings_fc = arcpy.GetParameterAsText(3)
    output_gdb = arcpy.GetParameterAsText(4)
    start_xy_text = arcpy.GetParameterAsText(5) #746699,382215
    end_xy_text = arcpy.GetParameterAsText(6) #747907,383991
    altitude_offset_text = arcpy.GetParameterAsText(7)
    api_key = "1d490f98beea0754ca8a15bfc848c685"

    start_xy = parse_xy(start_xy_text)
    end_xy = parse_xy(end_xy_text)
    altitude_offset = float(altitude_offset_text) if altitude_offset_text else 30.0 # domyslna wysokosc drona w metrach jezeli nie poda sie wysokosci
    result = compute_path(
        workspace,
        nmt_raster,
        buildings_fc,
        output_gdb,
        start_xy,
        end_xy,
        api_key,
        altitude_offset=altitude_offset,
        vegetation_raster=nmpt_raster,
    )
    arcpy.AddMessage(f"Wynik zapisany: {result}")


if __name__ == "__main__":
    main()