class AbstractTool():
    def __init__(self,port):
        self.port_ = port
    def abstract_methods(self):
        print('methods')


class ExtruderTool(AbstractTool):
    def __init__(self,port,tooltype):
        super().__init__(port)

        self.tooltype_ = tooltype

    def unique_methods(self):
        print('more methods')
        self.abstract_methods()


def main():
    fdmtool = ExtruderTool('ttyUSB0','FDM')
    fdmtool.unique_methods()


if __name__ == '__main__':
    main()