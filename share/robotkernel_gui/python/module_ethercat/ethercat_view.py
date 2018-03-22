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
import sys
import traceback
import gtk
import gobject
import yaml

from numpy import *

import links_and_nodes as ln

import gui_utils
import plugins
import ethercat_canopen_subview
import helpers

class module_ethercat_view(gui_utils.builder_ui):
    def __init__(self, parent, container, show_group):
        self.parent = parent
        self.app = parent.app

        gui_utils.builder_ui.__init__(self, filename="base.ui")

        self.canopen_view = ethercat_canopen_subview.ethercat_canopen_subview(self.app)
        self.parent.module_notebook.append_page(self.canopen_view.main, gtk.Label("CoE"))
        self.sercos_view = plugins.modules.module_sercos.sercos_id_subview.sercos_id_subview(self, "self.name", self.app)
        self.parent.module_notebook.append_page(self.sercos_view.tab_main, gtk.Label("SoE"))

        self.hide()

    def show(self, modname, module):
        self.canopen_view.main.show()
        self.sercos_view.tab_main.show()
        self.canopen_view.fill_device_treeview(module)

    def hide(self):
        self.canopen_view.main.hide()
        self.sercos_view.tab_main.hide()

    #HELPER
    def get_selected_device(self, widget):
        model, iter = widget.get_selection().get_selected()
        return model[iter][2]

