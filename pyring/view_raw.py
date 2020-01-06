import pyrealtime as prt
from controller_lib import get_device_data


def main():
    get_device_data(show_plot=True)
    prt.LayerManager.session().run()


if __name__ == "__main__":
    main()
