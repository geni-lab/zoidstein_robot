#!/usr/bin/env python

import rospy
from hri_framework import TextToSpeechActionServer
from std_msgs.msg import String


class ZoidsteinTextToSpeechServer(TextToSpeechActionServer):

    def __init__(self):
        TextToSpeechActionServer.__init__(self)
        self.tts_pub = rospy.Publisher('itf_talk', String, queue_size=1)

    #TODO: call synthesis_finished
    #TODO: call send_feedback

    def synthesise_sentence(self, sentence):
        self.tts_pub.publish(sentence)
        self.synthesis_finished()

    #TODO: estimate duration of sentence
    def tts_subsentence_duration(self, sentence, start_word_index, end_word_index):
        """ Return the duration (how long it takes to speak, double) a subset of a sentence that will be synthesised.
            The subset of the sentence is given via the parameters: start_word_index and end_word_index.
        """
        return

    #TODO: Cancel synthesis of tts
    def cancel_tts_synthesis(self):
        """ Cancel tts synthesis if it is currently running. """
        return


if __name__ == '__main__':
    rospy.init_node('zoidstein_tts_server')
    server = ZoidsteinTextToSpeechServer()
    server.start()
    rospy.spin()