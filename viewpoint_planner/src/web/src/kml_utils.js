function kmlToString(kml) {
    var kmlString = "";
    for (var i in kml) {
        kmlString += kml[i] + "\n";
    }
    return kmlString;
}

function createKmlDocument(children) {
    var kmlDocument = [];
    kmlDocument.push('<?xml version="1.0" encoding="UTF-8"?>');
    kmlDocument.push('<kml xmlns="http://www.opengis.net/kml/2.2"');
    kmlDocument.push(' xmlns:gx="http://www.google.com/kml/ext/2.2">');
    kmlDocument.push('');
    kmlDocument.push("<Document>");
    kmlDocument = kmlDocument.concat(children);
    kmlDocument.push("</Document>");
    kmlDocument.push("</kml>");
    return kmlDocument;
}

function createKmlTour(name, playlist) {
    var kmlTour = [];
    kmlTour.push("<gx:Tour>");
    kmlTour.push("<name>" + name + "</name>");
    kmlTour.push("<gx:Playlist>");
    kmlTour = kmlTour.concat(playlist);
    kmlTour.push("</gx:Playlist>");
    kmlTour.push("</gx:Tour>");
    return kmlTour;
}

function createKmlFlyTo(duration, abstractView) {
    var kmlFlyTo = [];
    kmlFlyTo.push("<gx:FlyTo>");
    kmlFlyTo.push("<gx:duration>" + duration + "</gx:duration>");
    kmlFlyTo.push("<gx:flyToMode>smooth</gx:flyToMode>");
    kmlFlyTo = kmlFlyTo.concat(abstractView);
    kmlFlyTo.push("</gx:FlyTo>");
    return kmlFlyTo;
}

function createKmlCamera(options) {
    var kmlCamera = [];
    kmlCamera.push("<Camera>");
    kmlCamera.push("<longitude>" + options.longitude + "</longitude>");
    kmlCamera.push("<latitude>" + options.latitude + "</latitude>");
    kmlCamera.push("<altitude>" + options.altitude + "</altitude>");
    kmlCamera.push("<heading>" + options.heading + "</heading>");
    kmlCamera.push("<tilt>" + options.tilt + "</tilt>");
    kmlCamera.push("<roll>0</roll>");
    kmlCamera.push("<gx:horizFov>120</gx:horizFov>");
    //kmlCamera.push("<altitudeMode>relativeToGround</altitudeMode>");
    kmlCamera.push("<altitudeMode>absolute</altitudeMode>");
    kmlCamera.push("</Camera>");
    return kmlCamera;
}

function createKmlDuration(duration) {
    var kmlCamera = [];
    kmlCamera.push("<gx:Wait>");
    kmlCamera.push("<gx:duration>" + duration + "</gx:duration>");
    kmlCamera.push("</gx:Wait>");
    return kmlCamera;
}

function getElevationsForLocations(locations, handler) {
    var elevator = new google.maps.ElevationService();
    elevator.getElevationForLocations({
        'locations': locations
    }, function(results, status) {
        var elevations = [];
        if (status === 'OK') {
            // Retrieve the first result
            for (var i = 0; i < results.length; ++i) {
                if (results[i]) {
                    var elevation = results[i].elevation;
                    elevations.push(elevation);
                    // console.log("Elevation: " + elevation);
                }
                else {
                    console.error("Unable to retrieve elevation data");
                }
            }
            if (results.length == locations.length) {
                handler(elevations);
            }
        } else {
            console.error("Elevation service failed due to: " + status);
        }
    });
}

function downloadViewpointsAsKml() {
    // var locations = [];
    // for (var i = 0; i < manualViewpoints.length; ++i) {
    //     var viewpoint = manualViewpoints[i].viewpoint;
    //     locations.push(new google.maps.LatLng(viewpoint.lat, viewpoint.long));
    // }
    // getElevationsForLocations(locations, function (elevations) {
    // };
    var kmlPlaylist = [];
    for (var i = 0; i < manualViewpoints.length; ++i) {
        var doLookAt = $('#look-at-radio').is(':checked');
        if (doLookAt && lookAtPos == null) {
            console.error("No look-at position defined!");
            return;
        }
        // console.log(doLookAt);
        var viewpoint = manualViewpoints[i].viewpoint;

        var yaw = viewpoint.yaw;
        var pitch = viewpoint.pitch;
        if (doLookAt) {
            diff = latlngToXy(viewpoint, lookAtPos);
            // console.log("diff = " + diff.x + ", " + diff.y);
            yaw = toDegrees(Math.atan2(diff.y, diff.x));
            pitch = toDegrees(Math.atan2(lookAtPos.altitude - viewpoint.altitude, Math.sqrt(diff.x * diff.x + diff.y * diff.y)));
            // console.log("yaw = " + yaw);
        }

        var kmlCamera = createKmlCamera({
            latitude: viewpoint.lat,
            longitude: viewpoint.long,
            altitude: viewpoint.altitude,
            heading: 90 - yaw,
            tilt: 90 + pitch
        });
        var duration = 0.1;
        var kmlFlyTo = createKmlFlyTo(duration, kmlCamera);
        kmlPlaylist = kmlPlaylist.concat(kmlFlyTo);
        var kmlDuration = createKmlDuration(duration);
        kmlPlaylist = kmlPlaylist.concat(kmlDuration);
    }
    var kmlTour = createKmlTour("Tour", kmlPlaylist);
    var kmlDocument = createKmlDocument(kmlTour);
    var kmlString = kmlToString(kmlDocument);
    // console.log(kmlString);
    var url = 'data:text/json;charset=utf8,' + encodeURIComponent(kmlString);
    window.open(url, '_blank');
    window.focus();
}
