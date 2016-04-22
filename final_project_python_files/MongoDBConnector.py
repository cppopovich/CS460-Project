from pymongo import *
import json
from datetime import *

class MongoDBConnector():
    def __init__(self):
        self.__connection = None
        self.__database = None
        
    def connect(self,uri,db):
        self.__connection = MongoClient(uri)
        self.__database = self.__connection[db]
    
    def close(self):
        self.__connection.close()
    
    def insertIntoCollection(self,collection,json):
        self.__database[collection].insert(json)
    
    def getCollectionCount(self, collection):
        return self.__database[collection].count()
    
    def clearCollection(self,collection):
        self.__database[collection].remove({}) #removing all things
        
    
    def removeCollectionItem(self,collection,json):
        self.__database[collection].remove(json)
    
    def findInCollection(self,collection,json):
        return self.__database[collection].find(json)
    
    def findOneInCollection(self,collection,json):
        return self.__database[collection].findOne(json)
