"""MGRS grid parsing and UTM origin (shared with Lanelet map and LAS tools)."""

from pyproj import CRS, Transformer

_MGRS_BANDS = "CDEFGHJKLMNPQRSTUVWX"
_MGRS_COL_LETTERS = "ABCDEFGHJKLMNPQRSTUVWXYZ"
_MGRS_ROW_LETTERS = "ABCDEFGHJKLMNPQRSTUV"


def parse_mgrs(mgrs_string):
    """
    Parse an MGRS string into UTM zone, hemisphere, easting, and northing.

    Accepts both full MGRS coordinates (e.g. "33TWN3497211656") and base grid
    zone designators (e.g. "33TWN"), in which case easting/northing offsets are 0
    and the result is the origin of the 100 km square.

    Note: Southern hemisphere support is approximate and not fully validated.

    Returns:
        tuple: (zone, hemisphere, easting, northing, band)
    """
    mgrs_string = str(mgrs_string).strip()
    if len(mgrs_string) < 5:
        raise ValueError(f"MGRS string must be at least 5 characters, got {mgrs_string!r}")

    zone = int(mgrs_string[:2])
    band = mgrs_string[2]
    square = mgrs_string[3:5]
    coords = mgrs_string[5:]
    coord_len = len(coords) // 2
    easting_str = coords[:coord_len].ljust(5, "0")
    northing_str = coords[coord_len:].ljust(5, "0")
    easting_offset = int(easting_str)
    northing_offset = int(northing_str)

    col_letter, row_letter = square[0], square[1]

    set_number = (zone - 1) % 3
    col_index = _MGRS_COL_LETTERS.index(col_letter)
    col_base = (set_number * 8) % 24
    col_in_zone = (col_index - col_base) % 24
    easting_100km = col_in_zone + 1

    set_row = (zone - 1) % 2
    row_index = _MGRS_ROW_LETTERS.index(row_letter)
    if set_row == 1:
        row_index = (row_index - 5) % 20

    hemisphere = "north" if band >= "N" else "south"
    band_index = _MGRS_BANDS.index(band)

    raw_northing = row_index * 100000 + northing_offset

    if hemisphere == "north":
        band_start_approx = (band_index - 10) * 888889
        cycle = max(0, int((band_start_approx - raw_northing + 1000000) // 2000000))
        northing = raw_northing + cycle * 2000000
    else:
        northing = raw_northing
        if northing < 1000000:
            northing += 10000000

    easting = easting_100km * 100000 + easting_offset
    return zone, hemisphere, easting, northing, band


def mgrs_grid_origin_utm(mgrs_grid):
    """
    UTM easting/northing (meters) at the southwest corner of the 100 km MGRS cell.

    Args:
        mgrs_grid: At least 5 characters, e.g. ``\"32UQU\"`` or full MGRS string
            (only the leading grid zone designator is used if longer).

    Returns:
        (easting, northing) in the UTM zone of the cell.
    """
    head = str(mgrs_grid).strip()[:5]
    _, _, easting, northing, _ = parse_mgrs(head)
    return float(easting), float(northing)


def mgrs_to_wgs(mgrs_string):
    """Convert a full MGRS string to (lat, lon) WGS84."""
    zone, hemisphere, easting, northing, _ = parse_mgrs(mgrs_string)
    is_south = hemisphere == "south"
    utm_crs = CRS.from_dict({"proj": "utm", "zone": zone, "south": is_south, "datum": "WGS84"})
    wgs84_crs = CRS.from_epsg(4326)
    transformer = Transformer.from_crs(utm_crs, wgs84_crs, always_xy=True)
    lon, lat = transformer.transform(easting, northing)
    return (lat, lon)
