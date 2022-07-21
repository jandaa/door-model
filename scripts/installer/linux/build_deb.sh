
# Make directories
mkdir -p package/opt/power_side_door
mkdir -p package/usr/share/applications
mkdir -p package/usr/share/icons/hicolor/scalable/apps

# Build file
pyinstaller power_side_door.spec

# Copy over files
cp dist/power_side_door package/opt/power_side_door/
cp gui/power_side_door.svg package/usr/share/icons/hicolor/scalable/apps/
cp power_side_door.desktop package/usr/share/applications/

# set package permissions
find package/opt/power_side_door -type f -exec chmod 644 -- {}
find package/opt/power_side_door -type d -exec chmod 755 -- {}
find package/usr/share -type f -exec chmod 644 -- {}
chmod +x package/opt/power_side_door/power_side_door

# Clear old deb file if it already exists
rm power_side_door.deb

# Make deb package
fpm -C package -s dir -t deb -n "power_side_door" -v 0.1.0 -p power_side_door.deb

# Install using:
# sudo dpkg -i power_side_door.deb