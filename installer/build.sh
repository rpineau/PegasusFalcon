#!/bin/bash

PACKAGE_NAME="PegasusFalcon_X2.pkg"
BUNDLE_NAME="org.rti-zone.PegasusFalconX2"

if [ ! -z "$app_id_signature" ]; then
    codesign -f -s "$app_id_signature" --verbose ../build/Release/libPegasusFalcon.dylib
fi

mkdir -p ROOT/tmp/PegasusFalcon_X2/
cp "../PegasusFalcon.ui" ROOT/tmp/PegasusFalcon_X2/
cp "../PegasusAstro.png" ROOT/tmp/PegasusFalcon_X2/
cp "../rotatorlist PegasusFalcon.txt" ROOT/tmp/PegasusFalcon_X2/
cp "../build/Release/libPegasusFalcon.dylib" ROOT/tmp/PegasusFalcon_X2/

if [ ! -z "$installer_signature" ]; then
	# signed package using env variable installer_signature
	pkgbuild --root ROOT --identifier $BUNDLE_NAME --sign "$installer_signature" --scripts Scripts --version 1.0 $PACKAGE_NAME
	pkgutil --check-signature ./${PACKAGE_NAME}

else
    pkgbuild --root ROOT --identifier $BUNDLE_NAME --scripts Scripts --version 1.0 $PACKAGE_NAME
fi

rm -rf ROOT
