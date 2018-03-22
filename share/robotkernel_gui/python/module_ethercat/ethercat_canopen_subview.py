'''
(C) Robert Burger <robert.burger@dlr.de>

This file is part of Robotkernel-GUI.

Robotkernel-GUI is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Robotkernel-GUI is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Robotkernel-GUI.  If not, see <http://www.gnu.org/licenses/>.
'''

import os
import gtk
import gobject

from plugins import *
from helpers import canopen

class ethercat_canopen_subview(gui_utils.builder_ui, canopen.treeview):
    def __init__(self, app):
        gui_utils.builder_ui.__init__(self, filename="dual_list.ui")

        self.app = app
        self.clnt = self.app.clnt
        self.active_color = self.app.window.get_style().text[0].to_string()
        self.init_gui()

    def init_gui(self):
        # treeviews
        self.create_device_treeview()
        self.devices.add(self.treeview_devices)
        self.treeview_devices.connect("cursor-changed", self.on_treeview_devices_cursor_changed)

        self.create_treeview(gtk.TreeView())
        self.values.add(self.treeview_dictionary)

        self.main.show_all()

    #CREATORS
    def create_device_treeview(self):
        self.liststore_devices = store = gtk.ListStore(str, str, gobject.TYPE_PYOBJECT) # name, data
        self.treeview_devices = view = gtk.TreeView(store)
        view.set_model(store)
        view.insert_column(gtk.TreeViewColumn("Slave", gtk.CellRendererText(), text=0), -1)
        view.insert_column(gtk.TreeViewColumn("Name", gtk.CellRendererText(), text=1), -1)
        view.connect("cursor-changed", self.on_treeview_devices_cursor_changed)
        store.set_sort_column_id(0, 0)

    def fill_device_treeview(self, module):
        self.liststore_devices.clear()

        #initially fill all devices found by ln
        for s in module.childs:
            def add(prefix, sort_key, devname):
                try:
                    ethercat_device = canopen.canopen_device(prefix, self.app, self.treeview_dictionary, module.name, devname)
                    name = ethercat_device.read_element(index=4104, sub_index=0).value
                    self.liststore_devices.append( (sort_key, name, ethercat_device) )
                except:
                    import traceback
                    print traceback.format_exc()
                    pass

            number = int(s.split('_')[-1])

            for t in ['mailbox', 'eeprom']:
                devname = '.'.join([s, t])
                sort_key = '%s %d' % (t, number)
                gobject.timeout_add(10, add, module.robotkernel_name, sort_key, devname)


    #HELPER
    def get_selected_device(self, widget):
        model, iter = widget.get_selection().get_selected()
        return model[iter][2] #device_id, device_name, pyobject

    #CALLBACKS
    def on_treeview_devices_cursor_changed(self, widget):
        dev = self.get_selected_device(widget)
        self.treestore_dictionary.clear()
        ids = dev.list_dictionary()
        map(lambda x: self.treestore_dictionary.insert(None, -1, [x, "", dev]), ids)

