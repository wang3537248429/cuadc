from __future__ import annotations

import html
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from .config import WMS_PORT
from .mission import ne_to_latlon
from .field_layout import FieldLayout, load_field_layout
from .route_state import load_route_state, public_areas


LAYER_NAME = "cuadc_field"
MAX_IMAGE_DIMENSION = 2048


def _field_bounds() -> tuple[float, float, float, float]:
    layout = load_field_layout()
    south_lat, west_lon = ne_to_latlon(
        -layout.field_size_m / 2.0,
        -layout.field_size_m / 2.0,
        layout.origin_lat_deg,
        layout.origin_lon_deg,
    )
    north_lat, east_lon = ne_to_latlon(
        layout.field_size_m / 2.0,
        layout.field_size_m / 2.0,
        layout.origin_lat_deg,
        layout.origin_lon_deg,
    )
    return west_lon, south_lat, east_lon, north_lat


def capabilities_xml(service_url: str) -> bytes:
    min_lon, min_lat, max_lon, max_lat = _field_bounds()
    endpoint = html.escape(service_url, quote=True)
    return f'''<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE WMT_MS_Capabilities SYSTEM "http://schemas.opengis.net/wms/1.1.1/WMS_MS_Capabilities.dtd">
<WMT_MS_Capabilities version="1.1.1">
  <Service><Name>OGC:WMS</Name><Title>CUADC Local Field</Title>
    <OnlineResource xmlns:xlink="http://www.w3.org/1999/xlink" xlink:type="simple" xlink:href="{endpoint}"/>
  </Service>
  <Capability>
    <Request>
      <GetCapabilities><Format>application/vnd.ogc.wms_xml</Format><DCPType><HTTP><Get><OnlineResource xmlns:xlink="http://www.w3.org/1999/xlink" xlink:type="simple" xlink:href="{endpoint}"/></Get></HTTP></DCPType></GetCapabilities>
      <GetMap><Format>image/png</Format><DCPType><HTTP><Get><OnlineResource xmlns:xlink="http://www.w3.org/1999/xlink" xlink:type="simple" xlink:href="{endpoint}"/></Get></HTTP></DCPType></GetMap>
    </Request>
    <Exception><Format>application/vnd.ogc.se_xml</Format></Exception>
    <Layer><Title>CUADC Local Field</Title><SRS>EPSG:4326</SRS>
      <LatLonBoundingBox minx="{min_lon:.8f}" miny="{min_lat:.8f}" maxx="{max_lon:.8f}" maxy="{max_lat:.8f}"/>
      <Layer queryable="0" opaque="1"><Name>{LAYER_NAME}</Name><Title>CUADC public field and live flight-controller route</Title><SRS>EPSG:4326</SRS>
        <BoundingBox SRS="EPSG:4326" minx="{min_lon:.8f}" miny="{min_lat:.8f}" maxx="{max_lon:.8f}" maxy="{max_lat:.8f}"/>
      </Layer>
    </Layer>
  </Capability>
</WMT_MS_Capabilities>
'''.encode("utf-8")


def render_map(
    bbox: tuple[float, float, float, float],
    width: int,
    height: int,
    state: dict | None = None,
) -> bytes:
    min_lon, min_lat, max_lon, max_lat = bbox
    if min_lon >= max_lon or min_lat >= max_lat:
        raise ValueError("BBOX must have increasing longitude and latitude")
    if not 1 <= width <= MAX_IMAGE_DIMENSION or not 1 <= height <= MAX_IMAGE_DIMENSION:
        raise ValueError(f"WIDTH and HEIGHT must be within 1..{MAX_IMAGE_DIMENSION}")
    image = np.full((height, width, 3), (58, 104, 52), dtype=np.uint8)
    state = state or load_route_state()
    try:
        layout = FieldLayout(**state.get("field_layout", {}))
    except (TypeError, ValueError):
        layout = load_field_layout()

    def pixel(north_m: float, east_m: float) -> tuple[int, int]:
        lat, lon = ne_to_latlon(
            north_m,
            east_m,
            layout.origin_lat_deg,
            layout.origin_lon_deg,
        )
        x = int(round((lon - min_lon) / (max_lon - min_lon) * (width - 1)))
        y = int(round((max_lat - lat) / (max_lat - min_lat) * (height - 1)))
        return x, y

    def polygon_for_square(north: float, east: float, half: float) -> np.ndarray:
        return np.array(
            [
                pixel(north - half, east - half),
                pixel(north - half, east + half),
                pixel(north + half, east + half),
                pixel(north + half, east - half),
            ],
            dtype=np.int32,
        )

    def polygon_for_rectangle(
        north: float,
        east: float,
        half_along: float,
        half_cross: float,
        heading_deg: float,
    ) -> np.ndarray:
        angle = math.radians(heading_deg)
        points = []
        for along, cross in (
            (-half_along, -half_cross),
            (-half_along, half_cross),
            (half_along, half_cross),
            (half_along, -half_cross),
        ):
            dn = along * math.cos(angle) - cross * math.sin(angle)
            de = along * math.sin(angle) + cross * math.cos(angle)
            points.append(pixel(north + dn, east + de))
        return np.array(points, dtype=np.int32)

    areas = public_areas(layout.to_dict(), str(state.get("group", "A")))
    landing = areas["takeoff_landing"]
    landing_poly = polygon_for_rectangle(
        float(landing["center_north_m"]),
        float(landing["center_east_m"]),
        float(landing["half_length_m"]),
        float(landing["half_width_m"]),
        float(landing["heading_deg"]),
    )
    cv2.fillPoly(image, [landing_poly], (96, 118, 82))
    cv2.polylines(image, [landing_poly], True, (240, 240, 240), 2, cv2.LINE_AA)
    for zone in areas["target_zones"]:
        zone_poly = polygon_for_rectangle(
            float(zone["center_north_m"]),
            float(zone["center_east_m"]),
            float(zone["half_size_m"]),
            float(zone["half_size_m"]),
            float(zone.get("heading_deg", 0.0)),
        )
        color = (196, 114, 35) if zone["name"] == "A" else (48, 70, 205)
        overlay = image.copy()
        cv2.fillPoly(overlay, [zone_poly], color)
        cv2.addWeighted(overlay, 0.25, image, 0.75, 0.0, image)
        cv2.polylines(image, [zone_poly], True, color, 2, cv2.LINE_AA)
        center = pixel(float(zone["center_north_m"]), float(zone["center_east_m"]))
        cv2.putText(image, str(zone["name"]), center, cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)

    center_px = pixel(0.0, 0.0)
    edge_px = pixel(0.0, float(areas["max_home_distance_m"]))
    radius_px = max(1, abs(edge_px[0] - center_px[0]))
    cv2.circle(image, center_px, radius_px, (180, 200, 180), 1, cv2.LINE_AA)

    mission = [
        (float(point["north_m"]), float(point["east_m"]))
        for point in state.get("mission", [])
        if point.get("north_m") is not None and point.get("east_m") is not None
    ]
    if len(mission) >= 2:
        cv2.polylines(
            image,
            [np.array([pixel(*point) for point in mission], dtype=np.int32)],
            False,
            (235, 215, 80),
            2,
            cv2.LINE_AA,
        )
    trajectory = state.get("trajectory", [])
    if len(trajectory) >= 2:
        cv2.polylines(
            image,
            [np.array([pixel(float(p["north_m"]), float(p["east_m"])) for p in trajectory], dtype=np.int32)],
            False,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
    for detection in state.get("detections", []):
        point = pixel(float(detection["north_m"]), float(detection["east_m"]))
        cv2.circle(image, point, 5, (0, 230, 255), -1, cv2.LINE_AA)
    aircraft = state.get("aircraft", {})
    if aircraft.get("lat_deg"):
        point = pixel(float(aircraft["north_m"]), float(aircraft["east_m"]))
        cv2.drawMarker(image, point, (20, 20, 255), cv2.MARKER_TRIANGLE_UP, 14, 2, cv2.LINE_AA)
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError("failed to encode WMS PNG")
    return encoded.tobytes()


def _service_exception(message: str) -> bytes:
    return (
        '<?xml version="1.0"?><ServiceExceptionReport version="1.1.1">'
        f"<ServiceException>{html.escape(message)}</ServiceException>"
        "</ServiceExceptionReport>"
    ).encode("utf-8")


class WmsHandler(BaseHTTPRequestHandler):
    server_version = "CUADC-WMS/1.1.1"

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        parsed = urlparse(self.path)
        if parsed.path not in ("/", "/wms"):
            self.send_error(404)
            return
        query = {key.upper(): values[-1] for key, values in parse_qs(parsed.query).items()}
        request = query.get("REQUEST", "GetCapabilities").upper()
        try:
            if request == "GETCAPABILITIES":
                body = capabilities_xml(f"http://127.0.0.1:{WMS_PORT}/wms?")
                content_type = "application/vnd.ogc.wms_xml"
            elif request == "GETMAP":
                if query.get("VERSION", "1.1.1") != "1.1.1":
                    raise ValueError("only WMS 1.1.1 is supported")
                if query.get("SRS", "").upper() != "EPSG:4326":
                    raise ValueError("only SRS=EPSG:4326 is supported")
                if query.get("FORMAT", "").lower() != "image/png":
                    raise ValueError("only FORMAT=image/png is supported")
                if query.get("LAYERS", "") != LAYER_NAME:
                    raise ValueError(f"LAYERS must be {LAYER_NAME}")
                bbox_values = tuple(float(value) for value in query["BBOX"].split(","))
                if len(bbox_values) != 4:
                    raise ValueError("BBOX must contain four numbers")
                body = render_map(
                    bbox_values,
                    int(query["WIDTH"]),
                    int(query["HEIGHT"]),
                )
                content_type = "image/png"
            else:
                raise ValueError(f"unsupported REQUEST={request}")
        except (KeyError, TypeError, ValueError, RuntimeError) as exc:
            body = _service_exception(str(exc))
            content_type = "application/vnd.ogc.se_xml"
            self.send_response(400)
        else:
            self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format_string: str, *args) -> None:
        print(f"[WMS] {self.address_string()} {format_string % args}", flush=True)


def main() -> int:
    server = ThreadingHTTPServer(("127.0.0.1", WMS_PORT), WmsHandler)
    print(f"[WMS] serving {LAYER_NAME} on http://127.0.0.1:{WMS_PORT}/wms", flush=True)
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0
