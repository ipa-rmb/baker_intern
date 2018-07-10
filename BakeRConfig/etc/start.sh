source ~/.bashrc

#miracenter BakeRConfig:etc/BakeR-application.xml -v MCFFile=BakeRConfig:etc/maps/Kenter/map.mcf,cleaningMap=BakeRConfig:etc/maps/Kenter/map-clean.xml,segmentationMap=BakeRConfig:etc/maps/Kenter/map-segmentation.xml,useNogo=true

miracenter BakeRConfig:etc/BakeR-application.xml -v MCFFile=EnvironmentMaps:maps/AVH2/map.mcf,cleaningMap=BakeRConfig:etc/maps/AVH2/map-clean.xml,segmentationMap=BakeRConfig:etc/maps/AVH2/map-segmentation.xml,useNogo=true WaypointVisitor:etc/WaypointVisitor.xml &

ssh demo@192.168.5.2 /localhome/demo/Desktop/start_baker.sh &

sleep 15s

miracd AutomaticaGUI
while [ true ] ; do
	python python/gui/app.py
done;
