# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import time
from .globals import *
from .shape import get_links, download_file
import os
import omni.ui as ui

WIDGET_WIDTH = 130

# this function is used to make sure the user can log into shapenet.org.  It should be used before creating the pickle.
def try_login(username, password):
    import webbot

    b = webbot.Browser(showWindow=False)
    b.go_to("shapenet.org/account")
    b.type(username, into="username")
    b.type(password, into="password")
    b.click("Sign in")
    time.sleep(1)
    # if we are logged in we should be able to re-open the page and see congratulations!
    b.refresh()
    time.sleep(1)
    page_source = b.get_page_source()
    login_success = page_source.find("Congratulations!") > -1
    b.quit()

    return login_success


# This is the base script to check if the user has a login and gather the CVS files from
# http://shapenet.cs.stanford.edu/shapenet/obj-zip/ShapeNetCore.v1/ to save locally
def save_v1_csvs(username, password, save_path):

    import urllib.request

    if try_login(username, password):
        if not os.path.exists(save_path):
            os.makedirs(save_path)
        url = g_shapenet_url + "1/"
        file_count_zero = 57
        files = ["04379243","03593526","04225987","02958343","02876657","04460130","03001627","02871439","02942699","02691156","03642806","02801938","03991062","04256520","03624134","02946921","04090263","04468005","03761084","03938244","03636649","02747177","03710193","04530566","03790512","03207941","02828884","03948459","04099429","03691459","03337140","02773838","02933112","02818832","02843684","03211117","03928116","03261776","04401088","04330267","03759954","02924116","03797390","04074963","02808440","02880940","03085013","03467517","04554684","02834778","03325088","04004475","02954340"]
        for index, href in enumerate(files):
            print(f"{file_count_zero} --Downloading {href} to {save_path}.")
            file_count_zero = file_count_zero - 1
            download_file(save_path + href, url + href + ".csv")
        return True
    else:
        print("Please go to shapenet.org and get a valid login.")
        return False


# this helper function creates a synsetDB entry from a shapenet v1 cvs file.
def create_synsetDBEntry(csv_file):
    import csv

    readCSV = csv.reader(csv_file, delimiter=",")
    skipFirst = True
    synsetDb = {}
    for row in readCSV:
        if skipFirst:
            skipFirst = False
            continue
        modelId = row[0]
        modelDb = modelId[: modelId.find(".")]
        modelId = modelId[modelId.find(".") + 1 :]

        wnsynset = row[1]
        wnlemmas = row[2]
        up = row[3]
        front = row[4]
        name = row[5]
        tags = row[6]

        synsetDb[modelId] = (wnsynset, wnlemmas, up, front, name, tags)
    return synsetDb


# This is the script used to create the shapenet_db2.pickle.bz2 file if the user already has shapenet v1 downloaded.
def create_db_from_files(path):
    import glob

    csv_files = glob.glob(path + "/*.csv")

    snDb = {}
    for filename in csv_files:
        synsetId = filename[-12:-4]

        with open(filename, encoding="utf8") as csv_file:
            snDb[synsetId] = create_synsetDBEntry(csv_file)

    return snDb


# save and test the pickled databse.
def save_and_testDB(snDb, out_file):
    # simple sanity check to make sure the input database is valid so we don't write out a crap one.
    if not len(snDb) == 57:
        return False

    import bz2

    try:
        import cPickle as pickle
    except:
        import pickle
    sfile = bz2.BZ2File(out_file, "wb")
    pickle.dump(snDb, sfile)
    sfile.close()
    f = bz2.BZ2File(out_file, "rb")

    new_dict = pickle.load(f)
    f.close()
    if len(new_dict) == 57:
        print("ID Database created successfully!")
        return True
    else:
        print("Failed to create ID Database :(")
        return False


class ShapenetLogin:
    def __init__(self, shapenetMenu):
        self._shapenetMenu = shapenetMenu
        self._models = {}
        self._login_window = None
        self.create_login_window()

    def _on_login_fn(self, widget):
        csv_location = get_local_shape_loc() + "/v1_csv/"
        username = self._username.model.get_value_as_string()
        password = self._password.model.get_value_as_string()
        logged_in = False
        if len(username) > 0 and len(password) > 0:
            logged_in = save_v1_csvs(username, password, csv_location)
        else:
            self._login_window.visible = False
            flags = ui.WINDOW_FLAGS_NO_RESIZE | ui.WINDOW_FLAGS_MODAL
            flags |= ui.WINDOW_FLAGS_NO_SCROLLBAR
            self.invalid_window = ui.Window("Username or Password invalid.", width=500, height=0, flags=flags)
            with self.invalid_window.frame:
                with ui.VStack(name="root", style={"VStack::root": {"margin": 10}}, height=0, spacing=20):
                    ui.Label("Pelase enter a valid user and password.", alignment=ui.Alignment.LEFT, word_wrap=True)
            self.invalid_window.visible = True

        if not logged_in:
            print(f"Attempting to use local files if they already exist in {csv_location}.")
        snDb = create_db_from_files(csv_location)
        if save_and_testDB(snDb, get_local_shape_loc() + g_pickle_file_name):
            self._shapenetMenu._hide_login_show_add()
        if pickle_file_exists():
            self._login_window.visible = False

    def create_login_window(self):
        flags = ui.WINDOW_FLAGS_NO_RESIZE | ui.WINDOW_FLAGS_MODAL
        flags |= ui.WINDOW_FLAGS_NO_SCROLLBAR
        self._login_window = ui.Window("Create Shapenet Database Index File", width=500, height=0, flags=flags)
        with self._login_window.frame:
            with ui.VStack():
                with ui.HStack(height=20):
                    ui.Label("Username or Email: ", alignment=ui.Alignment.CENTER, width=WIDGET_WIDTH)
                    self._username = ui.StringField()
                    ui.Spacer(width=6)
                ui.Spacer(height=10)
                with ui.HStack(height=20):
                    ui.Label("Password:", alignment=ui.Alignment.CENTER, width=WIDGET_WIDTH)
                    self._password = ui.StringField()
                    self._password.password_mode = True
                ui.Spacer(height=10)
                with ui.HStack(height=20):
                    ui.Button("Sign In to shapenet.org", clicked_fn=lambda b=None: self._on_login_fn(b))
                with ui.VStack(name="root", style={"VStack::root": {"margin": 10}}, height=0, spacing=20):
                    password_message = "You password will not be stored by Isaac Sim, it is only used to log into the shapenet.org web page.  Password encription is up to the shapenet.org web page."
                    ui.Label(password_message, alignment=ui.Alignment.LEFT, word_wrap=True)
