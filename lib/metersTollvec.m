function ll = metersTollvec(xy)
% Converts XY points from Spherical Mercator EPSG:900913 to lat/lon in WGS84 Datum
    originShift = 2 * pi * 6378137 / 2.0;
    lon = (xy(:,1) ./ originShift) * 180;
    lat = (xy(:,2) ./ originShift) * 180;
    lat = 180 / pi * (2 * atan( exp( lat * pi / 180)) - pi / 2);
    ll = [lon,lat];
end