#!/usr/bin/python

import os
import rospy
import rospkg
from std_msgs.msg import String
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *

def callback_sphinx_audio(msg):
    global decoder, in_speech_bf, pub_recognized
    decoder.process_raw(msg.data, False, False)
    if decoder.get_in_speech() != in_speech_bf:
        in_speech_bf = decoder.get_in_speech()
        if not in_speech_bf:
            decoder.end_utt()
            if decoder.hyp() != None:
                print("SpRec.->Recognized: " + "'" + decoder.hyp().hypstr + "'")
                pub_recognized.publish(decoder.hyp().hypstr)
            decoder.start_utt()

def main():
    global decoder, in_speech_bf, pub_recognized
    print("INITIALIZING SPEECH RECOGNITION WITH POCKETSPHINX AND JSGF GRAMMAR BY MARCOSOFT...")
    rospy.init_node("sp_rec")
    pub_recognized = rospy.Publisher("/recognized", String, queue_size=10)
    rospack = rospkg.RosPack()

    in_speech_bf = False
    l_model   = ""
    hmm_folder= "/usr/local/lib/python2.7/dist-packages/pocketsphinx/model/en-us/"
    dict_file = rospack.get_path("sprec_pocketsphinx") + "/vocab/gpsr.dic"
    gram_file = rospack.get_path("sprec_pocketsphinx") + "/vocab/gpsr.gram"
    gram_rule = "simple_command"
    gram_name = "gpsr_gram"
    if rospy.has_param("~hmm"):
        hmm_folder = rospy.get_param("~hmm")
    if rospy.has_param("~lm"):
        l_model = rospy.get_param("~lm")
    if rospy.has_param("~dict"):
        dict_file = rospy.get_param("~dict")
    if rospy.has_param("~gram"):
        gram_file = rospy.get_param("~gram")
    if rospy.has_param("~rule"):
        gram_rule = rospy.get_param("~rule")
    if rospy.has_param("~grammar"):
        gram_name = rospy.get_param("~grammar")

    print("SpRec.->Loading decoder with default config...")
    config = Decoder.default_config()
    config.set_string('-hmm', hmm_folder)
    config.set_string('-dict', dict_file)
    print("SpRec.->Initializing decoder using grammar: " + gram_file)
    decoder = Decoder(config)
    jsgf = Jsgf(gram_file)
    gram = gram_file[:len(gram_file)-5]
    print("SpRec.->Initializing jsgf grammar using rule: " + gram + '.' + gram_rule)
    rule = jsgf.get_rule(gram_name + '.' + gram_rule)
    fsg  = jsgf.build_fsg(rule, decoder.get_logmath(), 7.5)
    print("SpRec.->Writing fsg to " + gram + '.fsg')
    fsg.writefile(gram + '.fsg')
    decoder.set_fsg(gram, fsg)
    decoder.set_search(gram)
    decoder.start_utt()
    print("SpRec.->Decoder started successfully")
    rospy.Subscriber("/sphinx_audio", String, callback_sphinx_audio)
    rospy.spin()

if __name__ == "__main__":
    main()
