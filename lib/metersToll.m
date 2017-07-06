function  ll = metersToll(xy)
% Converts XY point from Spherical Mercator EPSG:900913 to lat/lon in WGS84 Datum
    x = xy(:,1);
    y = xy(:,2);
    originShift = 2 * pi * 6378137 / 2.0; % 20037508.342789244
    lon = (x ./ originShift) * 180;
    lat = (y ./ originShift) * 180;
    lat = 180 / pi * (2 * atan( exp( lat * pi / 180)) - pi / 2);
    ll = [lon,lat];
end