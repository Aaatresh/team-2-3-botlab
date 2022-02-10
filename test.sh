make
wait 
lcm-logplayer data/convex_10mx10m_5cm.log & 
./bin/slam --localization-only data/convex_10mx10m_5cm.map --action-only
