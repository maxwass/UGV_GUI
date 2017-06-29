function xy = llToMeters(lon, lat)
% Converts given lat/lon in WGS84 Datum to XY in Spherical Mercator EPSG:900913"
    originShift = 2 * pi * 6378137 / 2.0; % 20037508.342789244
    x = lon * originShift / 180;
    y = log(tan((90 + lat) * pi / 360 )) / (pi / 180);
    y = y * originShift / 180;
    xy = [x,y];

end