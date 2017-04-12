# location_manager
# Abstract
This is a location manager service project.

# Pre-request
You have to install sqlite lib first.<br/>
$ sudo apt-get install sqlite3 libsqlite3-dev <br/>
Find your libsqlite3.so path<br/>
In CMakelists.txt, change the target_link_libraries lib path to yours.<br/>

# Demo
$ rosrun location_manager locationMgr_server<br />
Please refer to locationMgr_client.cpp to get more detail about how to get the response.<br/>

