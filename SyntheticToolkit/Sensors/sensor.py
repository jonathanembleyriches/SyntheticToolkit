from abc import ABC, abstractmethod


class Sensor(ABC):
    """ """

    @abstractmethod
    def init_sensor(self, parent):
        """
        Fully initialises the sensor in Isaac. After this is called the sensor
        MUST be ready to yse.
        """

    ...

    @abstractmethod
    def sample(self): ...

    @abstractmethod
    def sample_save(self): ...

    @abstractmethod
    def sample_publish(self): ...
